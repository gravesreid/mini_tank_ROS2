import rclpy
from rclpy.node import Node
import time
from bmm150 import bmm150_I2C  # Assuming you have this module for interfacing with the sensor.
from std_msgs.msg import Float64  # Import Float64 message type.

I2C_BUS = 0x01  # default use I2C1
ADDRESS_3 = 0x13  # (CSB:1 SDO:1) default I2C address

class Bmm150Node(Node):
    def __init__(self):
        super().__init__('bmm150_node')
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.loop)
        self.bmm150 = bmm150_I2C(I2C_BUS, ADDRESS_3)
        
        # Create a publisher for Float64 message type.
        self.publisher_ = self.create_publisher(Float64, 'compass_degree', 10)
        
        self.setup()

    def setup(self):
        while self.bmm150.ERROR == self.bmm150.sensor_init():
            self.get_logger().info("sensor init error, please check connect")
            time.sleep(1)
        self.bmm150.set_operation_mode(self.bmm150.POWERMODE_NORMAL)
        self.bmm150.set_preset_mode(self.bmm150.PRESETMODE_HIGHACCURACY)
        self.bmm150.set_rate(self.bmm150.RATE_10HZ)
        self.bmm150.set_measurement_xyz()

        # Zero the sensor
        for i in range(10):
            self.get_heading()
            time.sleep(0.1)

    def read_sensor(self):
        geomagnetic = self.bmm150.get_geomagnetic()
        degree = self.bmm150.get_compass_degree()
        return degree, geomagnetic
    
    def get_heading(self):
        degree, _ = self.read_sensor()
        if degree > 180:
            degree -= 360
        return degree
    
    def loop(self):
        degree, _ = self.read_sensor()
        
        # Create a Float64 message for the degree.
        degree_msg = Float64()
        degree_msg.data = degree
        
        # Publish the message
        self.publisher_.publish(degree_msg)
        self.get_logger().info(f"Published compass degree: {degree}")

        time.sleep(0.1)
    

def main(args=None):
    rclpy.init(args=args)
    bmm150_node = Bmm150Node()
    try:
        rclpy.spin(bmm150_node)
    except KeyboardInterrupt:
        pass
    finally:
        bmm150_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



