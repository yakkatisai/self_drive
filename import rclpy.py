import serial
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Vector3  # Optional for combined publishing

class EncoderPublisher(Node):
    def __init__(self, port='/dev/ttyACM0', baud=9600):
        super().__init__('encoder_publisher')
        self.serial_port = serial.Serial(port, baud, timeout=1)
        time.sleep(2)  # Wait for Arduino reset
        self.serial_port.flushInput()

        # Publishers
        self.left_pub = self.create_publisher(Int32, 'left_encoder', 10)
        self.right_pub = self.create_publisher(Int32, 'right_encoder', 10)
        self.both_pub = self.create_publisher(Vector3, 'encoders', 10)

        # Timer for reading data
        self.timer = self.create_timer(0.05, self.read_serial_data)  # 20 Hz

    def parse_encoder_values(self, line):
        try:
            if "Left:" in line and "Right:" in line:
                left_part = line.split('|')[0].strip()
                right_part = line.split('|')[1].strip()

                left_count = int(left_part.split(':')[1].strip())
                right_count = int(right_part.split(':')[1].strip())
                return left_count, right_count
        except:
            pass
        return None, None

    def read_serial_data(self):
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode('utf-8').strip()
            left, right = self.parse_encoder_values(line)

            if left is not None and right is not None:
                # Publish separately
                self.left_pub.publish(Int32(data=left))
                self.right_pub.publish(Int32(data=right))

                # Publish combined
                vec = Vector3()
                vec.x = float(left)
                vec.y = float(right)
                self.both_pub.publish(vec)

                self.get_logger().info(f"Published -> Left: {left}, Right: {right}")
            else:
                self.get_logger().warn(f"Unparsed Line: {line}")

def main(args=None):
    rclpy.init(args=args)
    node = EncoderPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")
    finally:
        node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
