"""
LineTrack-v0 : Gymnasium wrapper around the Gazebo + ROS 2 simulation.

Observation (3-dim):
    [normalized_error, delta_error, current_linear_speed]

Action (2-dim, continuous [-1, 1]):
    [0] → linear velocity  mapped to [0.0 , 0.35] m/s
    [1] → angular velocity mapped to [-3.0, 3.0 ] rad/s

Domain Randomization (sim-to-real bridge):
    • Control latency   0-80 ms
    • Action noise      σ = 0.05
    • Sensor noise      σ = 0.05
"""

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
import time
import random
from collections import deque


# ---------------------------------------------------------------------------
# Lightweight CvBridge that avoids the cv_bridge C-extension import issues
# on machines without the ROS 2 C++ build (desktop training rigs, etc.).
# ---------------------------------------------------------------------------
class _SimpleCvBridge:
    @staticmethod
    def imgmsg_to_cv2(msg, desired_encoding="bgr8"):
        dtype = np.uint8
        if msg.encoding in ("bgr8", "rgb8"):
            img = np.frombuffer(msg.data, dtype=dtype).reshape(
                (msg.height, msg.width, 3)
            ).copy()
            if msg.encoding == "rgb8" and desired_encoding == "bgr8":
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif msg.encoding == "bgr8" and desired_encoding == "rgb8":
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            return img
        elif msg.encoding == "mono8":
            img = np.frombuffer(msg.data, dtype=dtype).reshape(
                (msg.height, msg.width)
            ).copy()
            if desired_encoding == "bgr8":
                return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            return img
        else:
            raise ValueError(f"Unsupported image encoding: {msg.encoding}")


class LineTrackEnv(gym.Env):
    """Gazebo ↔ ROS 2 Gymnasium environment for Robo404 line following."""

    metadata = {"render_modes": []}

    # A class-level counter so that multiple env instances get unique node names
    _instance_count = 0

    def __init__(self, render_mode=None):
        super().__init__()

        LineTrackEnv._instance_count += 1
        node_name = f"rl_env_{LineTrackEnv._instance_count}"

        # ── ROS 2 bootstrap ──────────────────────────────────────────────
        if not rclpy.ok():
            rclpy.init()
        self.node = Node(node_name)
        self.bridge = _SimpleCvBridge()

        # ── Spaces ───────────────────────────────────────────────────────
        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0], dtype=np.float32),
            high=np.array([1.0, 1.0], dtype=np.float32),
        )
        self.observation_space = spaces.Box(
            low=np.array([-1.5, -2.0, -0.5], dtype=np.float32),
            high=np.array([1.5, 2.0, 1.5], dtype=np.float32),
        )

        # ── ROS 2 pub/sub ────────────────────────────────────────────────
        self.cmd_pub = self.node.create_publisher(Twist, "/cmd_vel", 10)

        # QoS must match the publisher – Gazebo camera uses RELIABLE,
        # so we cannot use sensor_data (BEST_EFFORT) here.
        self.image_sub = self.node.create_subscription(
            Image,
            "/camera/image_raw",
            self._image_cb,
            10,                              # default RELIABLE QoS
        )
        self.odom_sub = self.node.create_subscription(
            Odometry, "/odom", self._odom_cb, 10
        )

        # ── Gazebo service clients ───────────────────────────────────────
        self._reset_sim  = self.node.create_client(Empty, "/reset_simulation")
        self._pause       = self.node.create_client(Empty, "/pause_physics")
        self._unpause     = self.node.create_client(Empty, "/unpause_physics")

        # ── Internal state ───────────────────────────────────────────────
        self.latest_image = None
        self.latest_linear_speed = 0.0

        self.current_error = 0.0
        self.last_error = 0.0
        self.line_lost_counter = 0
        self.step_counter = 0
        self.max_steps = 500           # longer episodes → better exploration

        # ── Domain Randomization ─────────────────────────────────────────
        self.dr_enabled = True
        self.latency_queue = deque(maxlen=10)
        self.action_noise_std  = 0.05
        self.sensor_noise_std  = 0.05
        self.latency_range_ms  = (0, 80)

    # =====================================================================
    # ROS callbacks
    # =====================================================================
    def _image_cb(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def _odom_cb(self, msg):
        self.latest_linear_speed = msg.twist.twist.linear.x

    # =====================================================================
    # Gazebo helpers
    # =====================================================================
    def _call(self, client):
        if not client.service_is_ready():
            client.wait_for_service(timeout_sec=2.0)
        if client.service_is_ready():
            client.call_async(Empty.Request())

    def _spin(self, duration_sec):
        t_end = time.time() + duration_sec
        while time.time() < t_end:
            rclpy.spin_once(self.node, timeout_sec=0.005)

    # =====================================================================
    # Vision  – extract normalised centroid error from the bottom camera
    # =====================================================================
    def _get_error(self):
        if self.latest_image is None:
            return 0.0, True

        img = self.latest_image
        h, w = img.shape[:2]

        # crop region identical to follower_node
        ch0, ch1 = h // 3, h
        cw0, cw1 = w // 6, 5 * w // 6
        crop = img[ch0:ch1, cw0:cw1]

        mask = cv2.inRange(crop, np.array([0, 0, 0]), np.array([30, 30, 30]))
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
        )

        best_area, best_M = 0, None
        for c in contours:
            M = cv2.moments(c)
            if M["m00"] > 500 and M["m00"] > best_area:
                best_area = M["m00"]
                best_M = M

        if best_M is None:
            return self.current_error, True  # line lost

        cx = cw0 + int(best_M["m10"] / best_M["m00"])
        return float(cx - w // 2) / float(w / 2), False

    # =====================================================================
    # step
    # =====================================================================
    def step(self, action):
        self.step_counter += 1

        # ── map tanh actions → physical velocities ───────────────────────
        lin = float((action[0] + 1.0) / 2.0 * 0.35)   # [0, 0.35] m/s
        ang = float(action[1] * 3.0)                    # [-3, 3] rad/s

        # ── domain randomisation ─────────────────────────────────────────
        if self.dr_enabled:
            lin += np.random.normal(0, self.action_noise_std)
            ang += np.random.normal(0, self.action_noise_std)
            lin = np.clip(lin, 0.0, 0.35)
            ang = np.clip(ang, -3.0, 3.0)
            delay = random.randint(*self.latency_range_ms) / 1000.0
            self.latency_queue.append((time.time() + delay, lin, ang))
        else:
            self.latency_queue.append((time.time(), lin, ang))

        # ── let the sim run one tick (~33 ms) ────────────────────────────
        self._call(self._unpause)
        t_end = time.time() + 0.033
        while time.time() < t_end:
            now = time.time()
            act_lin, act_ang = 0.0, 0.0
            for ts, l, a in self.latency_queue:
                if now >= ts:
                    act_lin, act_ang = l, a
            msg = Twist()
            msg.linear.x = act_lin
            msg.angular.z = act_ang
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.005)
        self._call(self._pause)

        # ── observation ──────────────────────────────────────────────────
        error, lost = self._get_error()
        if self.dr_enabled and not lost:
            error += np.random.normal(0, self.sensor_noise_std)
            error = np.clip(error, -1.0, 1.0)

        delta = error - self.current_error
        self.last_error = self.current_error
        self.current_error = error

        if lost:
            self.line_lost_counter += 1
        else:
            self.line_lost_counter = 0

        obs = np.array([error, delta, self.latest_linear_speed], dtype=np.float32)

        # ── reward ───────────────────────────────────────────────────────
        terminated = False
        truncated  = False

        if self.line_lost_counter > 15:
            reward = -20.0
            terminated = True
        else:
            r_center   = 1.0 - abs(error)                 # stay centred
            r_speed    = self.latest_linear_speed / 0.35   # go fast
            r_smooth   = -0.1 * (ang ** 2)                 # don't jerk
            reward = r_center * 0.6 + r_speed * 0.4 + r_smooth

        if self.step_counter >= self.max_steps:
            truncated = True

        return obs, float(reward), terminated, truncated, {}

    # =====================================================================
    # reset
    # =====================================================================
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        self._call(self._unpause)
        self.cmd_pub.publish(Twist())           # stop the car
        self._call(self._reset_sim)             # reset Gazebo
        self._spin(0.5)                         # let physics settle
        self._call(self._pause)

        self.current_error = 0.0
        self.last_error = 0.0
        self.line_lost_counter = 0
        self.step_counter = 0
        self.latency_queue.clear()

        error, _ = self._get_error()
        return np.array([error, 0.0, 0.0], dtype=np.float32), {}

    # =====================================================================
    # close
    # =====================================================================
    def close(self):
        self._call(self._unpause)
        self.cmd_pub.publish(Twist())
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
