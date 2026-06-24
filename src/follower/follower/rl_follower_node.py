import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
import cv2
import numpy as np
import onnxruntime
import os

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

class RlFollowerNode(Node):
    def __init__(self):
        super().__init__('rl_follower_node')
        
        # Declare parameters
        self.declare_parameter('model_path', '/home/jiucai/28th-conference-robo404_plus/models/sac_robo404.onnx')
        self.declare_parameter('min_area', 500)
        self.declare_parameter('start_enabled', False)
        # Camera QoS reliability: 'best_effort' (default, correct for most real
        # camera drivers) or 'reliable' (the Gazebo sim camera publishes
        # RELIABLE, so a best_effort subscriber receives nothing there).
        self.declare_parameter('image_reliability', 'best_effort')
        # Max linear speed the policy's [0,1] output maps to. The real robot's
        # safe range is ~0.05 m/s (default); raise it for sim visualization to
        # watch the car drive faster without retraining.
        self.declare_parameter('max_linear_speed', 0.05)

        self.model_path = self.get_parameter('model_path').value
        self.min_area = self.get_parameter('min_area').value
        self.should_move = self.get_parameter('start_enabled').value
        self.image_reliability = self.get_parameter('image_reliability').value
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        
        self.get_logger().info(f"Loading ONNX policy from: {self.model_path}")
        
        # Load ONNX model
        if not os.path.exists(self.model_path):
            self.get_logger().warn(f"ONNX model file not found at {self.model_path}! Inference will not work until training completes.")
            self.session = None
        else:
            self.session = onnxruntime.InferenceSession(self.model_path)
            self.get_logger().info("ONNX policy loaded successfully.")
            
        self.bridge = SimpleCvBridge()
        
        # Internal states
        self.latest_linear_speed = 0.0
        self.current_error = 0.0
        self.last_error = 0.0
        self.line_lost_counter = 0
        
        # Publishers and Subscribers
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel_line', 10)
        self.state_pub = self.create_publisher(String, '/path_state', 10)
        
        if self.image_reliability == 'reliable':
            image_qos = rclpy.qos.QoSProfile(
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                depth=10,
            )
        else:
            image_qos = rclpy.qos.qos_profile_sensor_data
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            image_qos
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Services
        self.start_service = self.create_service(Empty, 'start_follower', self.start_follower_callback)
        self.stop_service = self.create_service(Empty, 'stop_follower', self.stop_follower_callback)
        
    def start_follower_callback(self, request, response):
        self.get_logger().info("Follower started.")
        self.should_move = True
        return response
        
    def stop_follower_callback(self, request, response):
        self.get_logger().info("Follower stopped.")
        self.should_move = False
        return response
        
    def odom_callback(self, msg):
        self.latest_linear_speed = msg.twist.twist.linear.x
        
    def process_image(self, img):
        h, w = img.shape[:2]
        crop_h_start = 1 * h // 3
        crop_h_stop = h
        crop_w_start = w // 6
        crop_w_stop = 5 * w // 6
        
        crop = img[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop]
        mask = cv2.inRange(crop, np.array([0, 0, 0]), np.array([30, 30, 30]))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        valid_contours = []
        for c in contours:
            M = cv2.moments(c)
            if M['m00'] > self.min_area:
                valid_contours.append((M['m00'], M))
                
        if len(valid_contours) == 0:
            return self.current_error, True
            
        valid_contours.sort(key=lambda x: x[0], reverse=True)
        _, M = valid_contours[0]
        
        cx = crop_w_start + int(M["m10"] / M["m00"])
        error_px = cx - w // 2
        normalized_error = float(error_px) / float(w / 2)
        return normalized_error, False
        
    def image_callback(self, msg):
        # If the ONNX model is not loaded yet, try to load it now (in case it was built while node was running)
        if self.session is None:
            if os.path.exists(self.model_path):
                try:
                    self.session = onnxruntime.InferenceSession(self.model_path)
                    self.get_logger().info("ONNX policy loaded dynamically.")
                except Exception as e:
                    self.get_logger().error(f"Failed to load ONNX model dynamically: {e}")
                    return
            else:
                # Just publish zero commands if not loaded
                self.vel_pub.publish(Twist())
                self.state_pub.publish(String(data="LINE_LOST"))
                return
                
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return
            
        error, line_lost = self.process_image(cv_image)
        
        if line_lost:
            self.line_lost_counter += 1
        else:
            self.line_lost_counter = 0
            
        delta_error = error - self.current_error
        self.last_error = self.current_error
        self.current_error = error
        
        cmd_vel_msg = Twist()
        
        # If line is lost for too long (15 frames is ~0.5s), execute search spin
        if self.line_lost_counter > 15:
            path_state = "LINE_LOST"
            # Spin on the spot in direction of last known error
            angular_z = -1.5 * np.sign(self.current_error) if self.current_error != 0 else -1.2
            angular_z = max(-1.2, min(1.2, angular_z))
            
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = float(angular_z)
        else:
            path_state = "LINE_VISIBLE"
            
            # Predict actions using ONNX model
            obs = np.array([[error, delta_error, self.latest_linear_speed]], dtype=np.float32)
            inputs = {self.session.get_inputs()[0].name: obs}
            outputs = self.session.run(None, inputs)
            action = outputs[0][0]
            
            # Map action to physical velocities.
            # Policy was trained in sim with linear -> [0.0, 0.35] m/s, but the
            # real robot's usable range is ~0.03-0.05 m/s, so we proportionally
            # rescale the linear output into [0.0, max_linear_speed]. Angular is
            # left at the trained [-3.0, 3.0] rad/s range.
            linear_act = (action[0] + 1.0) / 2.0 * self.max_linear_speed
            angular_act = action[1] * 3.0
            
            cmd_vel_msg.linear.x = float(linear_act)
            cmd_vel_msg.angular.z = float(angular_act)
            
        self.state_pub.publish(String(data=path_state))
        
        if self.should_move:
            self.vel_pub.publish(cmd_vel_msg)
        else:
            self.vel_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = RlFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
