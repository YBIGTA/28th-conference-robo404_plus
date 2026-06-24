#!/usr/bin/env python3
"""
Gazebo auto-labelling recorder.

Drives a classical autopilot (e.g. the `follower`) around the sim track and
records frames that are FULLY auto-labelled from Gazebo ground truth:
  - precise 2D boxes (3D model pose -> camera projection, no hand-labelling)
  - true distance (object centre in camera frame)
  - traffic-light state  (red_light / green_light classes)
  - expert action        (the /cmd_vel being commanded)

Output is written by yolo_dataset.schema.DatasetWriter (YOLO labels + dataset.yaml
for training, frames.jsonl for the rest).

This node is written against the robo_description sim (camera at
/camera/image_raw, /camera/camera_info; diff-drive /cmd_vel) and Gazebo Classic
(/gazebo/model_states). It has NOT been run against a live Gazebo from here —
validate on the ROS machine.
"""
import os

import numpy as np
import rclpy
import yaml
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32, String

from yolo_dataset import projection as proj
from yolo_dataset.schema import DatasetWriter

try:
    from gazebo_msgs.msg import ModelStates
except ImportError:                       # let the rest import on machines w/o gazebo_msgs
    ModelStates = None


class RecorderNode(Node):
    def __init__(self):
        super().__init__("yolo_dataset_recorder")

        self.declare_parameter("output_dir", "robo404_dataset")
        self.declare_parameter("objects_config", "")
        self.declare_parameter("record_rate", 5.0)        # Hz, max frames saved per second
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("model_states_topic", "/model_states")
        self.declare_parameter("light_state_topic", "/traffic_light/state")
        self.declare_parameter("line_offset_topic", "/line_offset")
        self.declare_parameter("robot_model_name", "robo404")
        self.declare_parameter("min_distance_m", 0.2)     # ignore objects closer than this
        self.declare_parameter("max_distance_m", 8.0)     # ... or farther than this

        g = lambda n: self.get_parameter(n).value
        self.out_dir = g("output_dir")
        self.record_period = 1.0 / max(1e-3, g("record_rate"))
        self.robot_name = g("robot_model_name")
        self.min_d, self.max_d = g("min_distance_m"), g("max_distance_m")

        self._load_objects_config(g("objects_config"))
        self.writer = DatasetWriter(self.out_dir, self.class_names)

        try:
            from cv_bridge import CvBridge
            self.bridge = CvBridge()
        except (ImportError, SystemError):
            self.bridge = None
            self.get_logger().warn("cv_bridge unavailable - using numpy fallback")
        self.K = None
        self.model_states = None
        self.action = {"linear_x": 0.0, "angular_z": 0.0}
        self.light_state = "none"
        self.line_offset = None
        self._last_save = -1e9

        self.create_subscription(CameraInfo, g("camera_info_topic"), self._on_caminfo, 1)
        self.create_subscription(Twist, g("cmd_vel_topic"), self._on_cmd, 10)
        self.create_subscription(String, g("light_state_topic"), self._on_light, 10)
        self.create_subscription(Float32, g("line_offset_topic"), self._on_line, 10)
        if ModelStates is not None:
            self.create_subscription(ModelStates, g("model_states_topic"), self._on_models, 10)
        else:
            self.get_logger().warn("gazebo_msgs not found - cannot read ground-truth poses")
        self.create_subscription(Image, g("image_topic"), self._on_image, 5)

        self.get_logger().info(
            f"Recording to '{self.out_dir}'  classes={self.class_names}  "
            f"rate<={1.0/self.record_period:.1f}Hz")

    # ---------------- config ----------------
    def _load_objects_config(self, path):
        cfg = {}
        if path and os.path.exists(path):
            with open(path) as f:
                cfg = yaml.safe_load(f) or {}
        else:
            self.get_logger().warn(f"objects_config '{path}' not found - using built-in default")
        self.class_names = cfg.get("classes", ["red_light", "green_light", "obstacle"])
        # base_footprint -> camera_optical_link chain (defaults match robo_description URDF)
        chain = cfg.get("camera_chain", [
            {"xyz": [0, 0, 0.033], "rpy": [0, 0, 0]},
            {"xyz": [0.10, 0, 0.06], "rpy": [0, 0.5, 0]},
            {"xyz": [0, 0, 0], "rpy": [-1.5708, 0, -1.5708]},
        ])
        self.T_base_optical = proj.compose([(c["xyz"], c["rpy"]) for c in chain])
        # model-name substring -> {type/class, half_extents}
        self.objects = cfg.get("objects", {
            "traffic_light": {"type": "traffic_light", "half_extents": [0.05, 0.05, 0.15]},
            "obstacle": {"type": "static", "class": "obstacle", "half_extents": [0.1, 0.1, 0.1]},
        })

    # ---------------- subscriptions ----------------
    def _on_caminfo(self, msg):
        self.K = proj.intrinsics_from_camera_info(msg.k)

    def _on_cmd(self, msg):
        self.action = {"linear_x": float(msg.linear.x), "angular_z": float(msg.angular.z)}

    def _on_light(self, msg):
        s = msg.data.strip().lower()
        self.light_state = s if s in ("red", "green") else "none"

    def _on_line(self, msg):
        self.line_offset = float(msg.data)

    def _on_models(self, msg):
        self.model_states = msg

    def _on_image(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_save < self.record_period:
            return
        if self.K is None or self.model_states is None:
            return
        T_cam_world = self._camera_world_transform()
        if T_cam_world is None:
            return

        if self.bridge is not None:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        else:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            if msg.encoding == "rgb8":
                img = img[:, :, ::-1]
        h, w = img.shape[:2]
        detections = self._label_objects(T_cam_world, w, h)

        frame_id = f"{self.writer.count:06d}"
        import cv2
        cv2.imwrite(self.writer.image_path(frame_id), img)
        self.writer.write(frame_id, w, h, detections, self.light_state, self.action,
                          line_offset=self.line_offset, stamp=now)
        self._last_save = now
        if self.writer.count % 50 == 0:
            self.get_logger().info(f"{self.writer.count} frames recorded")

    # ---------------- labelling ----------------
    def _camera_world_transform(self):
        names = list(self.model_states.name)
        if self.robot_name not in names:
            return None
        pose = self.model_states.pose[names.index(self.robot_name)]
        p, q = pose.position, pose.orientation
        T_world_base = proj.make_tf([p.x, p.y, p.z], proj.quat_to_rot(q.x, q.y, q.z, q.w))
        T_world_cam = T_world_base @ self.T_base_optical
        return proj.invert_tf(T_world_cam)

    def _label_objects(self, T_cam_world, w, h):
        out = []
        for name, pose in zip(self.model_states.name, self.model_states.pose):
            if name == self.robot_name:
                continue
            spec = next((v for k, v in self.objects.items() if k in name), None)
            if spec is None:
                continue
            cls = self._class_for(spec)
            if cls is None:
                continue
            p, q = pose.position, pose.orientation
            box = proj.project_object(self.K, T_cam_world, [p.x, p.y, p.z],
                                      [q.x, q.y, q.z, q.w], spec["half_extents"], w, h)
            if box is None:
                continue
            if not (self.min_d <= box["distance_m"] <= self.max_d):
                continue
            box["class"] = cls
            out.append(box)
        return out

    def _class_for(self, spec):
        if spec.get("type") == "traffic_light":
            if self.light_state == "none":
                return None                       # don't label an unlit/unknown light
            return f"{self.light_state}_light"
        return spec.get("class")


def main(args=None):
    rclpy.init(args=args)
    node = RecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"Done: {node.writer.count} frames in '{node.out_dir}'")
        node.writer.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
