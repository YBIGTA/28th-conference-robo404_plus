import gymnasium as gym
from gymnasium import spaces
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
import cv2
from cv_bridge import CvBridge
import time
import random
from collections import deque

class Robo404Env(gym.Env):
    """
    A custom Gymnasium environment for the Robo404 robot line following task in Gazebo.
    """
    def __init__(self, node_name="robo404_rl_env"):
        super(Robo404Env, self).__init__()
        
        # Initialize ROS 2 node
        if not rclpy.ok():
            rclpy.init()
        self.node = Node(node_name)
        
        self.bridge = CvBridge()
        
        # Define action and observation spaces
        # Action: [target_linear_vel, target_angular_vel]
        # target_linear_vel range: [0.0, 0.35] m/s
        # target_angular_vel range: [-3.0, 3.0] rad/s
        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0], dtype=np.float32),
            high=np.array([1.0, 1.0], dtype=np.float32)
        )
        
        # Observation: [normalized_error, normalized_delta_error, current_linear_speed]
        # Normalized error range: [-1.0, 1.0] (corresponding to center_offset / half_width)
        self.observation_space = spaces.Box(
            low=np.array([-1.5, -2.0, -0.5], dtype=np.float32),
            high=np.array([1.5, 2.0, 1.5], dtype=np.float32)
        )
        
        # ROS 2 Publishers & Subscribers
        self.cmd_pub = self.node.create_publisher(Twist, "/cmd_vel", 10)
        
        self.image_sub = self.node.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            rclpy.qos.qos_profile_sensor_data
        )
        
        self.odom_sub = self.node.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )
        
        # Gazebo Client Services
        self.reset_sim_client = self.node.create_client(Empty, "/reset_simulation")
        self.pause_physics_client = self.node.create_client(Empty, "/pause_physics")
        self.unpause_physics_client = self.node.create_client(Empty, "/unpause_physics")
        
        # Internal State Variables
        self.latest_image = None
        self.latest_linear_speed = 0.0
        self.latest_angular_speed = 0.0
        
        self.current_error = 0.0
        self.last_error = 0.0
        self.line_lost_counter = 0
        self.step_counter = 0
        self.max_steps = 300
        
        # Domain Randomization Settings
        self.enable_domain_randomization = True
        self.latency_queue = deque(maxlen=10)
        self.action_noise_std = 0.05
        self.sensor_noise_std = 0.05
        self.latency_range_ms = (0, 80) # Simulate latency up to 80ms
        
    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
    def odom_callback(self, msg):
        self.latest_linear_speed = msg.twist.twist.linear.x
        self.latest_angular_speed = msg.twist.twist.angular.z
        
    def call_service(self, client):
        if not client.service_is_ready():
            # Wait briefly for service
            client.wait_for_service(timeout_sec=1.0)
        if client.service_is_ready():
            req = Empty.Request()
            client.call_async(req)
            
    def process_image(self):
        """Extract the line centroid error from the bottom camera image."""
        if self.latest_image is None:
            return 0.0, True # Line lost
            
        img = self.latest_image
        h, w = img.shape[:2]
        
        # Slit crop matching follower_node
        crop_h_start = 1 * h // 3
        crop_h_stop = h
        crop_w_start = w // 6
        crop_w_stop = 5 * w // 6
        
        crop = img[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop]
        
        # BGR black color range threshold
        mask = cv2.inRange(crop, np.array([0, 0, 0]), np.array([30, 30, 30]))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        valid_contours = []
        for c in contours:
            M = cv2.moments(c)
            if M['m00'] > 500: # MIN_AREA
                valid_contours.append((M['m00'], M))
                
        if len(valid_contours) == 0:
            return self.current_error, True # Line lost
            
        # Select largest contour
        valid_contours.sort(key=lambda x: x[0], reverse=True)
        _, M = valid_contours[0]
        
        # Calculate pixel center offset from image center
        cx = crop_w_start + int(M["m10"] / M["m00"])
        error_px = cx - w // 2
        
        # Normalize error to [-1.0, 1.0] range (approx based on half-width)
        normalized_error = float(error_px) / float(w / 2)
        return normalized_error, False
        
    def step(self, action):
        self.step_counter += 1
        
        # 1. Apply Action Domain Randomization
        linear_act = (action[0] + 1.0) / 2.0 * 0.35 # Map [-1,1] to [0, 0.35] m/s
        angular_act = action[1] * 3.0 # Map [-1,1] to [-3.0, 3.0] rad/s
        
        if self.enable_domain_randomization:
            # Add action noise
            linear_act += np.random.normal(0, self.action_noise_std)
            angular_act += np.random.normal(0, self.action_noise_std)
            linear_act = np.clip(linear_act, 0.0, 0.35)
            angular_act = np.clip(angular_act, -3.0, 3.0)
            
            # Simulate latency by queuing action
            latency_sec = random.randint(self.latency_range_ms[0], self.latency_range_ms[1]) / 1000.0
            self.latency_queue.append((time.time() + latency_sec, linear_act, angular_act))
        else:
            self.latency_queue.append((time.time(), linear_act, angular_act))
            
        # Unpause physics
        self.call_service(self.unpause_physics_client)
        
        # Spin ROS to process subscriptions and services
        t_end = time.time() + 0.033 # Run simulation step for ~33ms (sim-time equivalent)
        while time.time() < t_end:
            # Dequeue action whose latency has expired
            now = time.time()
            active_linear = 0.0
            active_angular = 0.0
            for item in list(self.latency_queue):
                if now >= item[0]:
                    active_linear = item[1]
                    active_angular = item[2]
                    
            msg = Twist()
            msg.linear.x = float(active_linear)
            msg.angular.z = float(active_angular)
            self.cmd_pub.publish(msg)
            
            rclpy.spin_once(self.node, timeout_sec=0.005)
            
        # Pause physics to compute observation and reward statically
        self.call_service(self.pause_physics_client)
        
        # 2. Get Observation
        error, line_lost = self.process_image()
        
        # Apply Sensor Domain Randomization
        if self.enable_domain_randomization and not line_lost:
            error += np.random.normal(0, self.sensor_noise_std)
            error = np.clip(error, -1.0, 1.0)
            
        delta_error = error - self.current_error
        self.last_error = self.current_error
        self.current_error = error
        
        # Update line lost counter
        if line_lost:
            self.line_lost_counter += 1
        else:
            self.line_lost_counter = 0
            
        observation = np.array([error, delta_error, self.latest_linear_speed], dtype=np.float32)
        
        # 3. Compute Reward
        reward = 0.0
        terminated = False
        truncated = False
        
        if self.line_lost_counter > 15: # Line lost for too many steps -> terminate
            reward = -20.0
            terminated = True
        else:
            # Reward staying close to center and moving fast
            reward_center = 1.0 - abs(error)
            reward_speed = self.latest_linear_speed / 0.35
            # Penalize wild angular actions
            reward_steering_penalty = -0.1 * (angular_act ** 2)
            
            reward = reward_center * 0.6 + reward_speed * 0.4 + reward_steering_penalty
            
        if self.step_counter >= self.max_steps:
            truncated = True
            
        return observation, float(reward), terminated, truncated, {}
        
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        
        self.call_service(self.unpause_physics_client)
        
        # Stop the robot first
        self.cmd_pub.publish(Twist())
        
        # Call Gazebo simulation reset
        self.call_service(self.reset_sim_client)
        
        # Let simulator settle for a few ticks
        t_end = time.time() + 0.5
        while time.time() < t_end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            
        # Pause physics
        self.call_service(self.pause_physics_client)
        
        # Reset internal variables
        self.current_error = 0.0
        self.last_error = 0.0
        self.line_lost_counter = 0
        self.step_counter = 0
        self.latency_queue.clear()
        
        # Get initial observation
        error, _ = self.process_image()
        observation = np.array([error, 0.0, 0.0], dtype=np.float32)
        
        return observation, {}
        
    def close(self):
        self.call_service(self.unpause_physics_client)
        self.cmd_pub.publish(Twist())
        self.node.destroy_node()
        rclpy.shutdown()
