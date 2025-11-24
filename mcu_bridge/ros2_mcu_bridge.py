import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import json

class MCUBridge(Node):
    def __init__(self):
        super().__init__('mcu_bridge')

        # Open serial connection to Teensy
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)

        # Subscribe to velocity commands => velocity commands are the keyboard + scripts
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg: Twist):
        # Extract linear velocity (forward/backward)
        forward = msg.linear.x
        
        # Extract turning (if needed)
        turn = msg.angular.z

        # Create JSON command
        cmd = {
            "type": "drive",
            "forward": forward,
            "turn": turn
        }

        # Send to MCU
        packet = json.dumps(cmd) + "\n"
        self.ser.write(packet.encode())

        self.get_logger().info(f"Sent to MCU: {cmd}")

def main(args=None):
    rclpy.init(args=args)
    node = MCUBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()