import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from yolo_msgs.msg import DetectionArray

from traffic_light.logic import (
    DetectionCandidate,
    STATE_UNKNOWN,
    TrafficLightConfig,
    analyze_traffic_light,
)


class TrafficLightNode(Node):
    def __init__(self):
        super().__init__("traffic_light_node")

        self.declare_parameter(
            "traffic_light_class_names",
            ["traffic light", "traffic_light"],
        )
        self.declare_parameter("min_detection_confidence", 0.5)
        self.declare_parameter("min_bbox_area_px", 25)
        self.declare_parameter("image_timeout_sec", 0.5)
        self.declare_parameter("red_h_low_1", 0)
        self.declare_parameter("red_h_high_1", 10)
        self.declare_parameter("red_h_low_2", 170)
        self.declare_parameter("red_h_high_2", 180)
        self.declare_parameter("red_s_min", 80)
        self.declare_parameter("red_v_min", 80)
        self.declare_parameter("green_h_low", 40)
        self.declare_parameter("green_h_high", 90)
        self.declare_parameter("green_s_min", 60)
        self.declare_parameter("green_v_min", 60)
        self.declare_parameter("min_red_ratio", 0.03)
        self.declare_parameter("min_green_ratio", 0.03)
        self.declare_parameter("red_green_margin", 1.2)

        self.config = self.load_config()
        self.image_timeout_sec = self.get_parameter("image_timeout_sec").value

        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_image_time = None
        self.last_state = None

        self.traffic_light_state_pub = self.create_publisher(
            String,
            "/traffic_light_state",
            10,
        )

        self.create_subscription(
            Image,
            "/camera/rgb/image_raw",
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            DetectionArray,
            "/yolo/detections",
            self.detections_callback,
            10,
        )

    def load_config(self):
        return TrafficLightConfig(
            traffic_light_class_names=tuple(
                self.get_parameter("traffic_light_class_names").value
            ),
            min_detection_confidence=self.get_parameter(
                "min_detection_confidence"
            ).value,
            min_bbox_area_px=self.get_parameter("min_bbox_area_px").value,
            red_h_low_1=self.get_parameter("red_h_low_1").value,
            red_h_high_1=self.get_parameter("red_h_high_1").value,
            red_h_low_2=self.get_parameter("red_h_low_2").value,
            red_h_high_2=self.get_parameter("red_h_high_2").value,
            red_s_min=self.get_parameter("red_s_min").value,
            red_v_min=self.get_parameter("red_v_min").value,
            green_h_low=self.get_parameter("green_h_low").value,
            green_h_high=self.get_parameter("green_h_high").value,
            green_s_min=self.get_parameter("green_s_min").value,
            green_v_min=self.get_parameter("green_v_min").value,
            min_red_ratio=self.get_parameter("min_red_ratio").value,
            min_green_ratio=self.get_parameter("min_green_ratio").value,
            red_green_margin=self.get_parameter("red_green_margin").value,
        )

    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.latest_image_time = self.now_seconds()

    def detections_callback(self, msg):
        if not self.has_recent_image():
            self.publish_state(STATE_UNKNOWN)
            return

        detections = [detection_to_candidate(detection) for detection in msg.detections]
        state = analyze_traffic_light(detections, self.latest_image, self.config)
        self.publish_state(state)

    def has_recent_image(self):
        if self.latest_image is None or self.latest_image_time is None:
            return False
        return self.now_seconds() - self.latest_image_time <= self.image_timeout_sec

    def publish_state(self, state):
        self.traffic_light_state_pub.publish(String(data=state))
        self.last_state = state

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9


def detection_to_candidate(detection):
    return DetectionCandidate(
        class_name=detection.class_name,
        score=detection.score,
        center_x=detection.bbox.center.position.x,
        center_y=detection.bbox.center.position.y,
        size_x=detection.bbox.size.x,
        size_y=detection.bbox.size.y,
    )


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
