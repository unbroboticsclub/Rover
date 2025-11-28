import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int32MultiArray
import serial
import json
import threading

class MCUBridge(Node):
    def __init__(self):
        super().__init__('mcu_bridge')

        # Open serial connection to Teensy (MUST NAME TEENSY BOARD, ttyACM0 is temporary)
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.01)

        # Subscribe to velocity commands => velocity commands are the keyboard + scripts
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publishers for data coming FROM the MCU
        self.encoder_pub = self.create_publisher(Int32MultiArray, '/encoders', 10)
        self.battery_pub = self.create_publisher(Float32, '/battery_voltage', 10)

        # Background thread that continuously reads data from the MCU
        self.serial_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
        self.serial_thread.start()

        self.get_logger().info("MCU bridge initialized and serial read thread running")

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

    def serial_read_loop(self):
        # This loop constantly reads sensor data sent FROM the MCU
        while rclpy.ok():
            try:
                line = self.ser.readline().decode().strip()
                if not line:
                    continue

                data = json.loads(line)

                # The MCU specifies what type of message it is
                msg_type = data.get("type", "")

                # Encoders
                if msg_type == "encoders":
                    msg = Int32MultiArray()
                    msg.data = [data["left"], data["right"]]
                    self.encoder_pub.publish(msg)

                # Battery voltage
                elif msg_type == "battery":
                    msg = Float32()
                    msg.data = data["voltage"]
                    self.battery_pub.publish(msg)

                # You can continue adding more sensor types here:
                # IMU, currents, temperatures, arm joints, etc.

            except json.JSONDecodeError:
                self.get_logger().warn(f"Bad JSON from MCU: {line}")
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MCUBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
