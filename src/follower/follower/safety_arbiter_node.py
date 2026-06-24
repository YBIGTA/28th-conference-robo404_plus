import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
import time

class SafetyArbiterNode(Node):
    def __init__(self):
        super().__init__('safety_arbiter_node')
        
        self.declare_parameter('proximity_threshold', 0.6)
        self.proximity_threshold = self.get_parameter('proximity_threshold').value
        
        self.latest_path_vel = Twist()
        self.latest_teleop_vel = Twist()
        self.latest_teleop_time = 0.0
        self.obstacle_active = False
        
        # Subscribe to Line Tracker Twist commands (Autonomous)
        self.path_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel_path',
            self.path_vel_callback,
            10
        )
        
        # Subscribe to WASD Keyboard Teleop Twist commands (Manual)
        self.teleop_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel_teleop',
            self.teleop_vel_callback,
            10
        )
        
        # Subscribe to YOLO detections (Safety sensor input)
        self.yolo_sub = self.create_subscription(
            Detection2DArray,
            '/yolo_detections',
            self.yolo_callback,
            10
        )
        
        # Publish to actual robot actuator control topic
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.get_logger().info("Safety Arbiter Node with prioritized override initialized and running.")

    def path_vel_callback(self, msg):
        self.latest_path_vel = msg
        self.publish_command()

    def teleop_vel_callback(self, msg):
        self.latest_teleop_vel = msg
        self.latest_teleop_time = time.time()
        self.publish_command()

    def yolo_callback(self, msg):
        # Scan detections for any obstacles that are too close
        close_obstacle_found = False
        for detection in msg.detections:
            for result in detection.results:
                # Only stop for obstacles, red traffic lights, or yellow traffic lights
                if result.id in ['obstacle_box', 'traffic_light_red', 'traffic_light_yellow']:
                    proximity = result.pose.pose.position.z
                    if proximity > self.proximity_threshold:
                        close_obstacle_found = True
                        self.get_logger().warn(
                            f"Obstacle '{result.id}' too close! Proximity: {proximity:.2f} (Threshold: {self.proximity_threshold:.2f}). Emergency Stop triggered."
                        )
                        break
            if close_obstacle_found:
                break
                
        self.obstacle_active = close_obstacle_found
        if self.obstacle_active:
            self.publish_command()

    def publish_command(self):
        cmd = Twist()
        if self.obstacle_active:
            # STOP commands (Emergency brake)
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            # Prioritize manual WASD keyboard controls if active in the last 1.0 second
            if time.time() - self.latest_teleop_time < 1.0:
                cmd.linear.x = self.latest_teleop_vel.linear.x
                cmd.angular.z = self.latest_teleop_vel.angular.z
            else:
                # Fall back to autonomous line tracking
                cmd.linear.x = self.latest_path_vel.linear.x
                cmd.angular.z = self.latest_path_vel.angular.z
            
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyArbiterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
