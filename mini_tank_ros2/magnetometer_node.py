import rclpy
from rclpy.node import Node
import sys
import os
import time
from bmm150 import *
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Vector3

I2C_BUS = 0x01  # default use I2C1
ADDRESS_3 = 0x13  # (CSB:1 SDO:1) default i2c address

class Bmm150Node(Node):
    def __init__(self):
        super().__init__('bmm150_node')
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.loop)
        self.bmm150 = bmm150_I2C(I2C_BUS, ADDRESS_3)
        
        # Create a publisher
        self.publisher_ = self.create_publisher(MagneticField, 'magnetometer', 10)
        
        self.setup()

    def setup(self):
        while self.bmm150.ERROR == self.bmm150.sensor_init():
            self.get_logger().info("sensor init error, please check connect")
            time.sleep(1)
        self.bmm150.set_operation_mode(self.bmm150.POWERMODE_NORMAL)
        self.bmm150.set_preset_mode(self.bmm150.PRESETMODE_HIGHACCURACY)
        self.bmm150.set_rate(self.bmm150.RATE_10HZ)
        self.bmm150.set_measurement_xyz()

    def loop(self):
        geomagnetic = self.bmm150.get_geomagnetic()
        magnetic_field_msg = MagneticField()
        magnetic_field_msg.magnetic_field = Vector3(x=float(geomagnetic[0]), y=float(geomagnetic[1]), z=float(geomagnetic[2]))
        # Since the MagneticField message also contains a header, we should populate it
        magnetic_field_msg.header.stamp = self.get_clock().now().to_msg()
        magnetic_field_msg.header.frame_id = "magnetometer_link"  # This should be the frame ID relevant to your setup
        
        # Publish the message
        self.publisher_.publish(magnetic_field_msg)
        self.get_logger().info(f"Published magnetometer data: {geomagnetic[0]}, {geomagnetic[1]}, {geomagnetic[2]}")

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


