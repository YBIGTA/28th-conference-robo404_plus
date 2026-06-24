import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from follower.line_tracker import LineTracker

class SimpleCvBridge:
    def imgmsg_to_cv2(self, msg, desired_encoding='bgr8'):
        if msg.encoding == 'bgr8':
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3)).copy()
            if desired_encoding == 'bgr8':
                return img
            elif desired_encoding == 'rgb8':
                return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        elif msg.encoding == 'rgb8':
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3)).copy()
            if desired_encoding == 'bgr8':
                return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif desired_encoding == 'rgb8':
                return img
        elif msg.encoding == 'mono8':
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width)).copy()
            if desired_encoding == 'bgr8':
                return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            elif desired_encoding == 'rgb8':
                return cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
            else:
                return img
        else:
            # Fallback/default handling
            img = np.frombuffer(msg.data, dtype=np.uint8).copy()
            if img.size == msg.height * msg.width * 3:
                img = img.reshape((msg.height, msg.width, 3))
                if desired_encoding == 'bgr8':
                    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                return img
            elif img.size == msg.height * msg.width:
                img = img.reshape((msg.height, msg.width))
                if desired_encoding == 'bgr8':
                    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                return img
            raise ValueError(f"Unsupported image encoding: {msg.encoding}")

class LineTrackerNode(Node):
    def __init__(self):
        super().__init__('line_tracker_node')
        
        # Declare parameters to make controller gains easily configurable
        self.declare_parameter('kp', 0.005)
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('min_area', 500)
        self.declare_parameter('threshold_val', 80)
        
        kp = self.get_parameter('kp').value
        linear_speed = self.get_parameter('linear_speed').value
        min_area = self.get_parameter('min_area').value
        threshold_val = self.get_parameter('threshold_val').value
        
        # Instantiate decoupled CV processor
        self.tracker = LineTracker(
            kp=kp,
            linear_speed=linear_speed,
            min_area=min_area,
            threshold_val=threshold_val
        )
        self.bridge = SimpleCvBridge()
        
        # Subscriptions (input)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publishers (output - strictly to intermediate topics as requested)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel_path', 10)
        self.state_pub = self.create_publisher(String, '/path_state', 10)
        
        self.get_logger().info("Line Tracker Node initialized.")

    def image_callback(self, msg):
        try:
            # Safely convert incoming ROS Image message to BGR OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        # Execute CV math logic externally
        angular_z, linear_x, status, _ = self.tracker.process_image(cv_image)

        # Publish Twist command message (to /cmd_vel_path)
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_x
        cmd_vel_msg.angular.z = angular_z
        self.vel_pub.publish(cmd_vel_msg)

        # Publish string state message (to /path_state)
        state_msg = String()
        state_msg.data = status
        self.state_pub.publish(state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LineTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
