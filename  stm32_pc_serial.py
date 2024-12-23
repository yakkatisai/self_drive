import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SerialSubscriber(Node):
    def __init__(self):
        super().__init__('serial_subscriber')
        self.subscription = self.create_subscription(
            String,
            'serial_topic',  # Make sure this matches the STM32 publisher topic
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Setup Serial port
        self.ser = serial.Serial('/dev/ttyACM0', 9600)  # Replace with your serial port and baudrate

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

    def read_from_serial(self):
        while rclpy.ok():
            line = self.ser.readline()  # Read a line from the serial port
            self.get_logger().info(f"Received: {line.decode().strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialSubscriber()

    # Read from serial in a separate loop
    node.read_from_serial()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
