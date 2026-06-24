import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty

from decision.logic import (
    DecisionInput,
    PATH_LINE_LOST,
    TRAFFIC_UNKNOWN,
    decide,
)


class DecisionNode(Node):
    def __init__(self):
        super().__init__("decision_node")

        self.declare_parameter("start_enabled", False)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("cmd_vel_line_timeout_sec", 0.5)
        self.declare_parameter("path_state_timeout_sec", 0.5)
        self.declare_parameter("traffic_light_timeout_sec", 1.0)

        self.motion_enabled = self.get_parameter("start_enabled").value
        self.publish_rate_hz = self.get_parameter("publish_rate_hz").value
        self.cmd_vel_line_timeout_sec = self.get_parameter(
            "cmd_vel_line_timeout_sec"
        ).value
        self.path_state_timeout_sec = self.get_parameter(
            "path_state_timeout_sec"
        ).value
        self.traffic_light_timeout_sec = self.get_parameter(
            "traffic_light_timeout_sec"
        ).value

        self.last_cmd_vel_line = Twist()
        self.path_state = PATH_LINE_LOST
        self.traffic_light_state = TRAFFIC_UNKNOWN
        self.red_latched = False

        self.last_cmd_vel_line_time = None
        self.last_path_state_time = None
        self.last_traffic_light_state_time = None

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.decision_state_pub = self.create_publisher(String, "/decision_state", 10)

        self.create_subscription(
            Twist,
            "/cmd_vel_line",
            self.cmd_vel_line_callback,
            10,
        )
        self.create_subscription(
            String,
            "/path_state",
            self.path_state_callback,
            10,
        )
        self.create_subscription(
            String,
            "/traffic_light_state",
            self.traffic_light_state_callback,
            10,
        )

        self.create_service(Empty, "/start_driving", self.start_driving_callback)
        self.create_service(Empty, "/stop_driving", self.stop_driving_callback)

        timer_period = 1.0 / max(float(self.publish_rate_hz), 0.1)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def cmd_vel_line_callback(self, msg):
        self.last_cmd_vel_line = msg
        self.last_cmd_vel_line_time = self.now_seconds()

    def path_state_callback(self, msg):
        self.path_state = msg.data.strip().upper()
        self.last_path_state_time = self.now_seconds()

    def traffic_light_state_callback(self, msg):
        self.traffic_light_state = msg.data.strip().upper()
        self.last_traffic_light_state_time = self.now_seconds()

    def start_driving_callback(self, request, response):
        self.motion_enabled = True
        return response

    def stop_driving_callback(self, request, response):
        self.motion_enabled = False
        self.publish_decision(Twist(), "IDLE")
        return response

    def timer_callback(self):
        now = self.now_seconds()
        traffic_light_state = self.effective_traffic_light_state(now)

        inputs = DecisionInput(
            motion_enabled=self.motion_enabled,
            path_state=self.path_state,
            traffic_light_state=traffic_light_state,
            red_latched=self.red_latched,
            cmd_vel_line_timed_out=self.is_timed_out(
                self.last_cmd_vel_line_time,
                self.cmd_vel_line_timeout_sec,
                now,
            ),
            path_state_timed_out=self.is_timed_out(
                self.last_path_state_time,
                self.path_state_timeout_sec,
                now,
            ),
        )
        result = decide(inputs)
        self.red_latched = result.red_latched

        if result.pass_cmd_vel_line:
            cmd_vel = copy_twist(self.last_cmd_vel_line)
        else:
            cmd_vel = Twist()

        self.publish_decision(cmd_vel, result.state)

    def effective_traffic_light_state(self, now):
        if self.is_timed_out(
            self.last_traffic_light_state_time,
            self.traffic_light_timeout_sec,
            now,
        ):
            return TRAFFIC_UNKNOWN
        return self.traffic_light_state

    def is_timed_out(self, last_time, timeout_sec, now):
        if last_time is None:
            return True
        return now - last_time > timeout_sec

    def publish_decision(self, cmd_vel, decision_state):
        self.cmd_vel_pub.publish(cmd_vel)
        self.decision_state_pub.publish(String(data=decision_state))

    def publish_stop(self):
        self.publish_decision(Twist(), "IDLE")

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9


def copy_twist(twist):
    copied = Twist()
    copied.linear.x = twist.linear.x
    copied.linear.y = twist.linear.y
    copied.linear.z = twist.linear.z
    copied.angular.x = twist.angular.x
    copied.angular.y = twist.angular.y
    copied.angular.z = twist.angular.z
    return copied


def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
