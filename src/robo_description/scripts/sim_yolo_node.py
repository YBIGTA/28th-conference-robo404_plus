#!/usr/bin/env python3
"""Sim-only YOLO traffic-light detector (x86 / CPU, no TensorRT).

Loads the finetuned ultralytics model (classes: red/green/off/yellow),
subscribes to the top camera image, and publishes:
  * /yolo/detections      (yolo_msgs/DetectionArray)  -> traffic_light_node
  * /yolo/dbg_image       (sensor_msgs/Image)         -> RViz annotated view

This replaces the Jetson TensorRT path (yolo_jetson) for desktop Gazebo
simulation, so the real finetuned weights actually run end to end.
"""
import glob
import os
import sys

# torch's bundled libgomp must be loaded before anything else pulls in more
# native libs, otherwise importing ultralytics fails on ARM with
# "cannot allocate memory in static TLS block". Re-exec once with LD_PRELOAD set.
if not os.environ.get("_SIM_YOLO_PRELOADED"):
    _candidates = glob.glob(
        os.path.expanduser(
            "~/.local/lib/python*/site-packages/torch.libs/libgomp-*.so*"
        )
    ) + glob.glob(
        "/usr/lib/python*/dist-packages/torch.libs/libgomp-*.so*"
    )
    if _candidates:
        preload = _candidates[0]
        existing = os.environ.get("LD_PRELOAD", "")
        os.environ["LD_PRELOAD"] = (
            preload + (":" + existing if existing else "")
        )
        os.environ["_SIM_YOLO_PRELOADED"] = "1"
        os.execv(sys.executable, [sys.executable] + sys.argv)

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from yolo_msgs.msg import Detection, DetectionArray

from ultralytics import YOLO


# BGR colours per class for the annotated debug image.
CLASS_COLORS = {
    "red": (0, 0, 255),
    "green": (0, 255, 0),
    "yellow": (0, 255, 255),
    "off": (128, 128, 128),
}


class SimYoloNode(Node):
    def __init__(self):
        super().__init__("sim_yolo_node")

        self.declare_parameter("model_path", "")
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("image_topic", "/camera/rgb/image_raw")
        self.declare_parameter("detections_topic", "/yolo/detections")
        self.declare_parameter("publish_debug_image", True)

        model_path = self.get_parameter("model_path").value
        self.conf = float(self.get_parameter("conf").value)
        image_topic = self.get_parameter("image_topic").value
        detections_topic = self.get_parameter("detections_topic").value
        self.publish_debug_image = bool(
            self.get_parameter("publish_debug_image").value
        )

        if not model_path or not os.path.exists(model_path):
            raise RuntimeError(f"model_path not found: {model_path!r}")

        self.get_logger().info(f"Loading finetuned YOLO model: {model_path}")
        self.model = YOLO(model_path)
        self.names = self.model.names
        self.get_logger().info(f"Model classes: {self.names}")

        self.bridge = CvBridge()
        self.det_pub = self.create_publisher(DetectionArray, detections_topic, 10)
        if self.publish_debug_image:
            self.dbg_pub = self.create_publisher(Image, "/yolo/dbg_image", 10)

        self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data
        )
        self.get_logger().info(f"sim_yolo_node ready (subscribed to {image_topic})")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        results = self.model.predict(frame, conf=self.conf, verbose=False)

        det_array = DetectionArray()
        det_array.header = msg.header
        annotated = frame.copy() if self.publish_debug_image else None

        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                xywh = box.xywh[0].cpu().numpy()
                cls_id = int(box.cls[0].cpu().numpy())
                score = float(box.conf[0].cpu().numpy())
                class_name = self.names.get(cls_id, str(cls_id))

                det = Detection()
                det.class_id = cls_id
                det.class_name = class_name
                det.score = score
                det.bbox.center.position.x = float(xywh[0])
                det.bbox.center.position.y = float(xywh[1])
                det.bbox.size.x = float(xywh[2])
                det.bbox.size.y = float(xywh[3])
                det_array.detections.append(det)

                if annotated is not None:
                    cx, cy, w, h = xywh
                    x1, y1 = int(cx - w / 2), int(cy - h / 2)
                    x2, y2 = int(cx + w / 2), int(cy + h / 2)
                    color = CLASS_COLORS.get(class_name, (255, 255, 255))
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(
                        annotated,
                        f"{class_name} {score:.2f}",
                        (x1, max(0, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        color,
                        2,
                    )

        self.det_pub.publish(det_array)
        if annotated is not None:
            dbg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            dbg.header = msg.header
            self.dbg_pub.publish(dbg)


def main(args=None):
    rclpy.init(args=args)
    node = SimYoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
