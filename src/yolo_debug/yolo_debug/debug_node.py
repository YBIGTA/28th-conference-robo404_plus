# Copyright 2023 Miguel Angel Gonzalez Santamarta
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

"""YOLO debug visualization node."""

import random
import time
from typing import Optional
from typing import Tuple

import cv2
import numpy as np

import message_filters
import rclpy
from cv_bridge import CvBridge
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import LifecycleState
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import Image
from yolo_msgs.msg import BoundingBox2D
from yolo_msgs.msg import Detection
from yolo_msgs.msg import DetectionArray


class DebugNode(LifecycleNode):
    """Render YOLO detections and optionally stream the debug image over UDP."""

    def __init__(self) -> None:
        """Initialize parameters and local state."""
        super().__init__("debug_node")

        self._class_to_color = {}
        self.cv_bridge = CvBridge()

        self._dbg_pub = None
        self._stream_writer: Optional[cv2.VideoWriter] = None
        self._stream_failed = False
        self._last_frame_time = None
        self._fps = 0.0

        self.declare_parameter("image_reliability", QoSReliabilityPolicy.BEST_EFFORT)
        self.declare_parameter("publish_dbg_image", False)
        self.declare_parameter("enable_udp_stream", True)
        self.declare_parameter("stream_host", "127.0.0.1")
        self.declare_parameter("stream_port", 5000)
        self.declare_parameter("stream_width", 960)
        self.declare_parameter("stream_height", 540)
        self.declare_parameter("stream_framerate", 30)
        self.declare_parameter("stream_bitrate", 4000000)
        self.declare_parameter("use_hw_encoder", True)
        self.declare_parameter("draw_fps", True)

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Create optional publishers and read runtime parameters."""
        self.get_logger().info(f"[{self.get_name()}] Configuring...")

        self.image_qos_profile = QoSProfile(
            reliability=self.get_parameter("image_reliability")
            .get_parameter_value()
            .integer_value,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        self.publish_dbg_image = self.get_parameter("publish_dbg_image").value
        self.enable_udp_stream = self.get_parameter("enable_udp_stream").value
        self.stream_host = self.get_parameter("stream_host").value
        self.stream_port = self.get_parameter("stream_port").value
        self.stream_width = self.get_parameter("stream_width").value
        self.stream_height = self.get_parameter("stream_height").value
        self.stream_framerate = self.get_parameter("stream_framerate").value
        self.stream_bitrate = self.get_parameter("stream_bitrate").value
        self.use_hw_encoder = self.get_parameter("use_hw_encoder").value
        self.draw_fps = self.get_parameter("draw_fps").value

        if not self.validate_stream_parameters():
            return TransitionCallbackReturn.FAILURE

        if self.publish_dbg_image:
            self._dbg_pub = self.create_publisher(Image, "dbg_image", 10)

        super().on_configure(state)
        self.get_logger().info(f"[{self.get_name()}] Configured")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Subscribe to synchronized image and detection topics."""
        self.get_logger().info(f"[{self.get_name()}] Activating...")

        self.image_sub = message_filters.Subscriber(
            self, Image, "image_raw", qos_profile=self.image_qos_profile
        )
        self.detections_sub = message_filters.Subscriber(
            self, DetectionArray, "detections", qos_profile=10
        )

        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            (self.image_sub, self.detections_sub), 10, 0.5
        )
        self._synchronizer.registerCallback(self.detections_cb)

        super().on_activate(state)
        self.get_logger().info(f"[{self.get_name()}] Activated")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Destroy subscriptions and close the stream writer."""
        self.get_logger().info(f"[{self.get_name()}] Deactivating...")

        if hasattr(self, "image_sub"):
            self.destroy_subscription(self.image_sub.sub)
        if hasattr(self, "detections_sub"):
            self.destroy_subscription(self.detections_sub.sub)
        if hasattr(self, "_synchronizer"):
            del self._synchronizer
        self.close_stream_writer()

        super().on_deactivate(state)
        self.get_logger().info(f"[{self.get_name()}] Deactivated")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Destroy optional publishers and close resources."""
        self.get_logger().info(f"[{self.get_name()}] Cleaning up...")

        if self._dbg_pub is not None:
            self.destroy_publisher(self._dbg_pub)
            self._dbg_pub = None
        self.close_stream_writer()

        super().on_cleanup(state)
        self.get_logger().info(f"[{self.get_name()}] Cleaned up")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Close resources before shutdown."""
        self.get_logger().info(f"[{self.get_name()}] Shutting down...")
        self.close_stream_writer()
        super().on_shutdown(state)
        self.get_logger().info(f"[{self.get_name()}] Shutted down")
        return TransitionCallbackReturn.SUCCESS

    def validate_stream_parameters(self) -> bool:
        """Validate stream parameters before the node activates."""
        if not self.enable_udp_stream:
            return True

        numeric_parameters = {
            "stream_port": self.stream_port,
            "stream_width": self.stream_width,
            "stream_height": self.stream_height,
            "stream_framerate": self.stream_framerate,
            "stream_bitrate": self.stream_bitrate,
        }

        for name, value in numeric_parameters.items():
            if value <= 0:
                self.get_logger().error("{} must be > 0".format(name))
                return False

        if not self.stream_host:
            self.get_logger().error("stream_host must not be empty")
            return False

        return True

    def draw_box(
        self,
        cv_image: np.ndarray,
        detection: Detection,
        color: Tuple[int, int, int],
    ) -> np.ndarray:
        """Draw a 2D bounding box with class name and confidence score."""
        box_msg: BoundingBox2D = detection.bbox
        min_pt = (
            round(box_msg.center.position.x - box_msg.size.x / 2.0),
            round(box_msg.center.position.y - box_msg.size.y / 2.0),
        )
        max_pt = (
            round(box_msg.center.position.x + box_msg.size.x / 2.0),
            round(box_msg.center.position.y + box_msg.size.y / 2.0),
        )

        min_pt = self.clamp_point(min_pt, cv_image)
        max_pt = self.clamp_point(max_pt, cv_image)
        cv2.rectangle(cv_image, min_pt, max_pt, color, 2)

        label = f"{detection.class_name} {detection.score:.3f}"
        text_pos = (min_pt[0] + 5, max(20, min_pt[1] + 25))
        cv2.putText(
            cv_image,
            label,
            text_pos,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            color,
            2,
            cv2.LINE_AA,
        )

        return cv_image

    def clamp_point(self, point: Tuple[int, int], cv_image: np.ndarray) -> Tuple[int, int]:
        """Clamp a point into image bounds."""
        image_height, image_width = cv_image.shape[:2]
        x = max(0, min(image_width - 1, point[0]))
        y = max(0, min(image_height - 1, point[1]))
        return x, y

    def class_color(self, class_name: str) -> Tuple[int, int, int]:
        """Return a stable BGR color for a class name."""
        if class_name not in self._class_to_color:
            rng = random.Random(class_name)
            self._class_to_color[class_name] = (
                rng.randint(0, 255),
                rng.randint(0, 255),
                rng.randint(0, 255),
            )
        return self._class_to_color[class_name]

    def draw_fps_label(self, cv_image: np.ndarray) -> np.ndarray:
        """Draw an exponentially smoothed callback FPS value."""
        now = time.monotonic()
        if self._last_frame_time is not None:
            elapsed = now - self._last_frame_time
            if elapsed > 0.0:
                instant_fps = 1.0 / elapsed
                if self._fps <= 0.0:
                    self._fps = instant_fps
                else:
                    self._fps = self._fps * 0.9 + instant_fps * 0.1
        self._last_frame_time = now

        if self._fps > 0.0:
            cv2.putText(
                cv_image,
                f"FPS {self._fps:.2f}",
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
        return cv_image

    def hardware_pipeline(self) -> str:
        """Return the Jetson hardware H.264 RTP pipeline."""
        return (
            "appsrc is-live=true block=false format=time do-timestamp=true"
            f" ! video/x-raw,format=BGR,width=(int){self.stream_width}"
            f",height=(int){self.stream_height}"
            f",framerate=(fraction){self.stream_framerate}/1"
            " ! queue leaky=downstream max-size-buffers=1"
            " ! videoconvert ! video/x-raw,format=I420"
            " ! nvvidconv"
            f" ! nvv4l2h264enc bitrate={self.stream_bitrate}"
            " insert-sps-pps=true"
            " ! h264parse"
            " ! rtph264pay config-interval=1 pt=96"
            f" ! udpsink host={self.stream_host} port={self.stream_port}"
            " sync=false async=false"
        )

    def software_pipeline(self) -> str:
        """Return the software H.264 RTP pipeline."""
        bitrate_kbps = max(1, int(self.stream_bitrate / 1000))
        return (
            "appsrc is-live=true block=false format=time do-timestamp=true"
            f" ! video/x-raw,format=BGR,width=(int){self.stream_width}"
            f",height=(int){self.stream_height}"
            f",framerate=(fraction){self.stream_framerate}/1"
            " ! queue leaky=downstream max-size-buffers=1"
            " ! videoconvert ! video/x-raw,format=I420"
            " ! x264enc tune=zerolatency speed-preset=ultrafast"
            f" bitrate={bitrate_kbps}"
            f" key-int-max={self.stream_framerate}"
            " ! rtph264pay config-interval=1 pt=96"
            f" ! udpsink host={self.stream_host} port={self.stream_port}"
            " sync=false async=false"
        )

    def open_stream_writer(self) -> None:
        """Open the UDP stream writer with hardware then software fallback."""
        if self._stream_writer is not None or self._stream_failed:
            return

        pipelines = []
        if self.use_hw_encoder:
            pipelines.append(("hardware", self.hardware_pipeline()))
        pipelines.append(("software", self.software_pipeline()))

        for pipeline_name, pipeline in pipelines:
            writer = cv2.VideoWriter(
                pipeline,
                cv2.CAP_GSTREAMER,
                0,
                float(self.stream_framerate),
                (self.stream_width, self.stream_height),
                True,
            )
            if writer.isOpened():
                self._stream_writer = writer
                self.get_logger().info(
                    "Opened {} debug stream to {}:{}".format(
                        pipeline_name,
                        self.stream_host,
                        self.stream_port,
                    )
                )
                return

            writer.release()
            self.get_logger().warn(
                "Failed to open {} debug stream pipeline".format(pipeline_name)
            )

        self._stream_failed = True
        self.get_logger().error("UDP debug stream disabled after pipeline failures")

    def close_stream_writer(self) -> None:
        """Close the UDP stream writer."""
        if self._stream_writer is not None:
            self._stream_writer.release()
            self._stream_writer = None

    def publish_stream(self, cv_image: np.ndarray) -> None:
        """Write one frame to the UDP stream."""
        if not self.enable_udp_stream or self._stream_failed:
            return

        self.open_stream_writer()
        if self._stream_writer is None:
            return

        if (
            cv_image.shape[1] != self.stream_width
            or cv_image.shape[0] != self.stream_height
        ):
            stream_image = cv2.resize(
                cv_image,
                (self.stream_width, self.stream_height),
                interpolation=cv2.INTER_LINEAR,
            )
        else:
            stream_image = cv_image

        self._stream_writer.write(np.ascontiguousarray(stream_image))

    def detections_cb(self, img_msg: Image, detection_msg: DetectionArray) -> None:
        """Render detections and publish selected debug outputs."""
        cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")

        for detection in detection_msg.detections:
            color = self.class_color(detection.class_name)
            cv_image = self.draw_box(cv_image, detection, color)

        if self.draw_fps:
            cv_image = self.draw_fps_label(cv_image)

        if self._dbg_pub is not None:
            self._dbg_pub.publish(
                self.cv_bridge.cv2_to_imgmsg(
                    cv_image, encoding="bgr8", header=img_msg.header
                )
            )

        self.publish_stream(cv_image)


def main():
    """Run the debug node."""
    rclpy.init()
    node = DebugNode()
    node.trigger_configure()
    node.trigger_activate()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_stream_writer()
        node.destroy_node()
        rclpy.shutdown()
