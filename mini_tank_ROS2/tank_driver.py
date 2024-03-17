import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist

class Driver(Node):

    def __init__(self, port='/dev/ttyACM0'):  # Adjust port as necessary
        super().__init__('cmd_vel_listener')
        self.ser = serial.Serial(
            port=port,
            baudrate=9600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.callback,
            10)

    def callback(self, msg):
        command = 'stop'  # Default command
        
        # Determine the command based on the velocity values
        if msg.linear.x > 0:
            command = 'forward'
        elif msg.linear.x < 0:
            command = 'back'
        elif msg.angular.z > 0:
            command = 'left'
        elif msg.angular.z < 0:
            command = 'right'
        
        self.serialWrite(command)

    def serialWrite(self, command):
        # Simplified method to send commands
        self.ser.write((command + '\n').encode())  # Add newline character to denote the end of a command
        
def main(args=None):
    rclpy.init(args=args)
    driver = Driver()
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


