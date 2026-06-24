import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from yolo_msgs.msg import DetectionArray

from debug_monitor.logic import (
    PipelineSnapshot,
    build_pipeline_state,
    build_warnings,
    format_warning_line,
    make_freshness,
    normalize_state,
    summarize_detections,
)


class DebugMonitorNode(Node):
    def __init__(self):
        super().__init__("debug_monitor_node")

        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("path_state_timeout_sec", 0.5)
        self.declare_parameter("cmd_vel_line_timeout_sec", 0.5)
        self.declare_parameter("yolo_detections_timeout_sec", 1.0)
        self.declare_parameter("traffic_light_state_timeout_sec", 1.0)
        self.declare_parameter("decision_state_timeout_sec", 0.5)
        self.declare_parameter("cmd_vel_timeout_sec", 0.5)
        self.declare_parameter("zero_twist_epsilon", 0.001)
        self.declare_parameter("cmd_compare_epsilon", 0.001)
        self.declare_parameter(
            "traffic_light_class_names",
            ["traffic light", "traffic_light"],
        )

        self.publish_rate_hz = self.get_parameter("publish_rate_hz").value
        self.zero_twist_epsilon = self.get_parameter("zero_twist_epsilon").value
        self.cmd_compare_epsilon = self.get_parameter("cmd_compare_epsilon").value
        self.traffic_light_class_names = self.get_parameter(
            "traffic_light_class_names"
        ).value

        self.timeouts = {
            "path_state": self.get_parameter("path_state_timeout_sec").value,
            "cmd_vel_line": self.get_parameter("cmd_vel_line_timeout_sec").value,
            "yolo_detections": self.get_parameter(
                "yolo_detections_timeout_sec"
            ).value,
            "traffic_light_state": self.get_parameter(
                "traffic_light_state_timeout_sec"
            ).value,
            "decision_state": self.get_parameter("decision_state_timeout_sec").value,
            "cmd_vel": self.get_parameter("cmd_vel_timeout_sec").value,
        }

        self.latest = {
            "path_state": {"msg": None, "time": None},
            "cmd_vel_line": {"msg": None, "time": None},
            "yolo_detections": {"msg": None, "time": None},
            "traffic_light_state": {"msg": None, "time": None},
            "decision_state": {"msg": None, "time": None},
            "cmd_vel": {"msg": None, "time": None},
        }

        self.pipeline_state_pub = self.create_publisher(
            String,
            "/debug/pipeline_state",
            10,
        )
        self.pipeline_warnings_pub = self.create_publisher(
            String,
            "/debug/pipeline_warnings",
            10,
        )

        self.create_subscription(String, "/path_state", self.path_state_callback, 10)
        self.create_subscription(Twist, "/cmd_vel_line", self.cmd_vel_line_callback, 10)
        self.create_subscription(
            DetectionArray,
            "/yolo/detections",
            self.yolo_detections_callback,
            10,
        )
        self.create_subscription(
            String,
            "/traffic_light_state",
            self.traffic_light_state_callback,
            10,
        )
        self.create_subscription(
            String,
            "/decision_state",
            self.decision_state_callback,
            10,
        )
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)

        timer_period = 1.0 / max(float(self.publish_rate_hz), 0.1)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def path_state_callback(self, msg):
        self.store_sample("path_state", msg)

    def cmd_vel_line_callback(self, msg):
        self.store_sample("cmd_vel_line", msg)

    def yolo_detections_callback(self, msg):
        self.store_sample("yolo_detections", msg)

    def traffic_light_state_callback(self, msg):
        self.store_sample("traffic_light_state", msg)

    def decision_state_callback(self, msg):
        self.store_sample("decision_state", msg)

    def cmd_vel_callback(self, msg):
        self.store_sample("cmd_vel", msg)

    def store_sample(self, name, msg):
        self.latest[name] = {"msg": msg, "time": self.now_seconds()}

    def timer_callback(self):
        snapshot = self.build_snapshot()
        warnings = build_warnings(
            snapshot,
            self.zero_twist_epsilon,
            self.cmd_compare_epsilon,
        )
        self.pipeline_state_pub.publish(
            String(
                data=build_pipeline_state(
                    snapshot,
                    warnings,
                    self.zero_twist_epsilon,
                    self.cmd_compare_epsilon,
                )
            )
        )
        self.pipeline_warnings_pub.publish(String(data=format_warning_line(warnings)))

    def build_snapshot(self):
        now = self.now_seconds()
        freshness = {
            name: make_freshness(
                name,
                self.latest[name]["time"],
                self.timeouts[name],
                now,
            )
            for name in self.latest
        }

        yolo_msg = self.latest["yolo_detections"]["msg"]
        detections = [] if yolo_msg is None else list(yolo_msg.detections)

        return PipelineSnapshot(
            path_state=string_data(self.latest["path_state"]["msg"]),
            traffic_light_state=string_data(self.latest["traffic_light_state"]["msg"]),
            decision_state=string_data(self.latest["decision_state"]["msg"]),
            cmd_vel=self.latest["cmd_vel"]["msg"],
            cmd_vel_line=self.latest["cmd_vel_line"]["msg"],
            detection_summary=summarize_detections(
                detections,
                self.traffic_light_class_names,
            ),
            freshness=freshness,
        )

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9


def string_data(msg):
    if msg is None:
        return normalize_state(None)
    return normalize_state(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = DebugMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
