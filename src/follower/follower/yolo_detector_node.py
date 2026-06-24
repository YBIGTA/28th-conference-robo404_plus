import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from follower.yolo_detector import YoloDetector

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

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        
        # Declare YOLO configuration parameter
        self.declare_parameter('model_path', 'yolov8n.pt')
        model_path = self.get_parameter('model_path').value
        
        # Instantiate decoupled CV/YOLO processor
        self.detector = YoloDetector(model_path=model_path, fallback_to_mock=True)
        self.bridge = SimpleCvBridge()
        
        # Save start time for simulated dynamic traffic light cycle
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        
        # Subscriptions (input)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publishers (output - strictly to intermediate topics as requested)
        self.detections_pub = self.create_publisher(
            Detection2DArray,
            '/yolo_detections',
            10
        )
        
        # Annotated image publisher for visualization integration
        self.dbg_image_pub = self.create_publisher(
            Image,
            '/yolo_detector_node/dbg_image',
            10
        )
        
        self.get_logger().info("YOLOv8 Detector Node initialized with dynamic sign cycling and visualization.")

    def image_callback(self, msg):
        try:
            # Convert incoming ROS Image message to BGR OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        # Execute CV/YOLO logic externally
        detections = self.detector.detect(cv_image)

        # Dynamic traffic light state cycle (10s GREEN, 10s RED)
        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.start_time
        active_light = 'green' if (elapsed % 20.0) < 10.0 else 'red'

        # Build vision_msgs/msg/Detection2DArray message
        array_msg = Detection2DArray()
        array_msg.header = msg.header

        debug_image = cv_image.copy()

        for det in detections:
            cls = det['class_name']
            
            # Filter detections based on active light state
            if cls == 'obstacle_box':
                pass
            elif cls == 'traffic_light_green' and active_light == 'green':
                pass
            elif cls == 'traffic_light_red' and active_light == 'red':
                pass
            elif cls == 'traffic_light_yellow' and active_light == 'yellow':
                pass
            else:
                continue

            det2d = Detection2D()
            det2d.header = msg.header

            # Build hypothesis details
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = cls
            hypothesis.score = det['confidence']
            
            # Pack virtual proximity score into Z coordinates of Pose to act as virtual distance
            hypothesis.pose.pose.position.z = det['proximity']
            
            det2d.results.append(hypothesis)

            # Build bounding box
            det2d.bbox.center.x = det['bbox'][0]
            det2d.bbox.center.y = det['bbox'][1]
            det2d.bbox.size_x = det['bbox'][2]
            det2d.bbox.size_y = det['bbox'][3]

            array_msg.detections.append(det2d)

            # Draw bounding box
            cx, cy, w, h = det['bbox']
            x1 = int(cx - w / 2)
            y1 = int(cy - h / 2)
            x2 = int(cx + w / 2)
            y2 = int(cy + h / 2)

            if 'red' in cls or 'obstacle' in cls:
                color = (0, 0, 255) # Red
            elif 'yellow' in cls:
                color = (0, 255, 255) # Yellow
            elif 'green' in cls:
                color = (0, 255, 0) # Green
            else:
                color = (255, 0, 0) # Blue

            cv2.rectangle(debug_image, (x1, y1), (x2, y2), color, 2)
            label = f"{cls} ({det['confidence']:.2f}, prox:{det['proximity']:.2f})"
            cv2.putText(debug_image, label, (x1, max(y1 - 5, 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Draw current dynamic traffic light cycle status on screen for debug
        status_text = f"Simulated Phase: {active_light.upper()}"
        cv2.putText(debug_image, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # Publish the detection results array (to /yolo_detections)
        self.detections_pub.publish(array_msg)

        # Publish annotated debug image
        try:
            dbg_msg = Image()
            dbg_msg.header = msg.header
            dbg_msg.height = debug_image.shape[0]
            dbg_msg.width = debug_image.shape[1]
            dbg_msg.encoding = 'bgr8'
            dbg_msg.is_bigendian = 0
            dbg_msg.step = debug_image.shape[1] * 3
            dbg_msg.data = debug_image.tobytes()
            self.dbg_image_pub.publish(dbg_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish debug image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
