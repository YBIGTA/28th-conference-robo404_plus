#!/usr/bin/env python3
"""Demo orchestrator for the robo404+ Gazebo presentation.

Flow:
  1. Wait briefly for the stack to settle, then call /start_driving.
  2. Watch /decision_state. When the car latches a STOP at a red light
     (STOP_FOR_RED / WAIT_GREEN), swap that red light prop for a green one
     at the same pose (delete_entity + spawn_entity). The traffic_light
     node then detects GREEN and the decision node releases the latch, so
     the car resumes.
  3. Each red light is handled once, in path order.

This gives a hands-off "drive -> stop at red -> light turns green -> go"
demo suitable for screen recording.
"""
import os
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Pose
from std_msgs.msg import String

from ament_index_python.packages import get_package_share_directory

from gen_world import LIGHTS, light_world_poses


def _green_model_sdf_path():
    """Locate traffic_light_green/model.sdf in either the install or source tree."""
    candidates = [
        os.path.join(
            get_package_share_directory("robo_description"),
            "models", "traffic_light_green", "model.sdf",
        ),
        os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "..", "models", "traffic_light_green", "model.sdf",
        ),
    ]
    for path in candidates:
        if os.path.exists(path):
            return path
    raise FileNotFoundError(
        "traffic_light_green/model.sdf not found in: " + ", ".join(candidates)
    )


GREEN_MODEL_SDF = _green_model_sdf_path()


class DemoOrchestrator(Node):
    def __init__(self):
        super().__init__("demo_orchestrator")
        self.declare_parameter("start_delay_sec", 4.0)
        self.declare_parameter("min_stop_sec", 1.5)
        self.start_delay = float(self.get_parameter("start_delay_sec").value)
        self.min_stop = float(self.get_parameter("min_stop_sec").value)

        with open(GREEN_MODEL_SDF) as f:
            self.green_sdf = f.read()

        # Red lights in path order, with their world poses.
        poses = light_world_poses()
        self.red_queue = [
            (cfg["name"], poses[cfg["name"]])
            for cfg in LIGHTS
            if "red" in cfg["model"]
        ]
        self.get_logger().info(
            f"Red lights to switch (in order): {[n for n, _ in self.red_queue]}"
        )

        self.start_cli = self.create_client(Empty, "/start_driving")
        self.del_cli = self.create_client(DeleteEntity, "/delete_entity")
        self.spawn_cli = self.create_client(SpawnEntity, "/spawn_entity")

        self.decision_state = None
        self.stop_since = None
        self.started = False
        self.t0 = time.time()
        self.create_subscription(
            String, "/decision_state", self._decision_cb, 10
        )
        self.timer = self.create_timer(0.2, self._tick)

    def _decision_cb(self, msg):
        self.decision_state = msg.data

    def _call(self, cli, req, what):
        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"service for {what} unavailable")
            return None
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return fut.result()

    def _tick(self):
        now = time.time()
        if not self.started:
            if now - self.t0 >= self.start_delay:
                self.get_logger().info("Calling /start_driving")
                self._call(self.start_cli, Empty.Request(), "start_driving")
                self.started = True
            return

        if not self.red_queue:
            return

        stopped = self.decision_state in ("STOP_FOR_RED", "WAIT_GREEN")
        if stopped:
            if self.stop_since is None:
                self.stop_since = now
            elif now - self.stop_since >= self.min_stop:
                self._switch_next_red()
                self.stop_since = None
        else:
            self.stop_since = None

    def _switch_next_red(self):
        name, (x, y, yaw) = self.red_queue.pop(0)
        self.get_logger().info(f"Car stopped at red -> switching {name} to GREEN")

        self._call(self.del_cli, DeleteEntity.Request(name=name), "delete")

        req = SpawnEntity.Request()
        req.name = name + "_green"
        req.xml = self.green_sdf
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.0
        qz, qw = _yaw_to_quat(yaw)
        pose.orientation.z = qz
        pose.orientation.w = qw
        req.initial_pose = pose
        self._call(self.spawn_cli, req, "spawn green")


def _yaw_to_quat(yaw):
    import math
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def main(args=None):
    rclpy.init(args=args)
    node = DemoOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
