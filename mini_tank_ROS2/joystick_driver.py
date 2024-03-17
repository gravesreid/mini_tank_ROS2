import rclpy
from rclpy.node import Node
import serial
from sensor_msgs.msg import Joy

class Driver(Node):

    def __init__(self, port='/dev/ttyUSB0'):  # Adjust port as necessary
        super().__init__('joy_listener')
        self.ser = serial.Serial(
            port=port,
            baudrate=9600,  # Make sure this matches your Arduino's baud rate
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.callback,
            10)

    def callback(self, msg):
        # Interpreting joystick axis for movement commands
        if msg.axes[1] > 0:
            self.serialWrite('forward')
        elif msg.axes[1] < 0:
            self.serialWrite('back')
        elif msg.axes[0] > 0:
            self.serialWrite('left')
        elif msg.axes[0] < 0:
            self.serialWrite('right')
        else:
            self.serialWrite('stop')  # Default to stop if no clear direction

    def serialWrite(self, command):
        # Simplified method to send commands as simple strings
        self.ser.write((command + '\n').encode())

def main(args=None):
    rclpy.init(args=args)
    driver = Driver()
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
