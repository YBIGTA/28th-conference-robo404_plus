import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import tty
import termios

msg = """
========================================
    Control Your Robot with WASD!
========================================
Keys:
        w : forward
   a : left   s : backward   d : right

   space key / x : force stop speed to 0.0

Speed increments:
   - w/s : change linear speed by +/- 0.05
   - a/d : change angular speed by +/- 0.1

Press Ctrl-C to quit.
========================================
"""

class WASDTeleopNode(Node):
    def __init__(self):
        super().__init__('wasd_teleop_node')
        self.pub = self.create_publisher(Twist, '/cmd_vel_teleop', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        
        # Poll stdin for keypresses at 50Hz (every 20ms)
        self.create_timer(0.02, self.keyboard_loop)
        print(msg)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = None
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def keyboard_loop(self):
        try:
            key = self.get_key()
            if key is not None:
                if key == 'w':
                    self.linear_speed = round(min(1.0, self.linear_speed + 0.05), 2)
                elif key == 's':
                    self.linear_speed = round(max(-1.0, self.linear_speed - 0.05), 2)
                elif key == 'a':
                    self.angular_speed = round(min(2.0, self.angular_speed + 0.1), 2)
                elif key == 'd':
                    self.angular_speed = round(max(-2.0, self.angular_speed - 0.1), 2)
                elif key in [' ', 'x']:
                    self.linear_speed = 0.0
                    self.angular_speed = 0.0
                elif ord(key) == 3: # Ctrl-C
                    raise KeyboardInterrupt
                    
                # Publish the manual teleop velocities
                twist = Twist()
                twist.linear.x = float(self.linear_speed)
                twist.angular.z = float(self.angular_speed)
                self.pub.publish(twist)
                print(f"\rCurrent Manual Target -> Linear: {self.linear_speed:+.2f} m/s | Angular: {self.angular_speed:+.2f} rad/s    ", end="")
        except KeyboardInterrupt:
            # Send stop command before exiting
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.pub.publish(twist)
            print("\nExiting WASD Keyboard Teleop.")
            rclpy.shutdown()
            sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = WASDTeleopNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass

if __name__ == '__main__':
    main()
