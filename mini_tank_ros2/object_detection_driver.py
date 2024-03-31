#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import struct
import sys
import time
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from bittle_msgs.msg import Detection



class Driver(Node):

    def __init__(self, port='/dev/ttyUSB0'):
        super().__init__('cmd_vel_listener')
        self.subscription = self.create_subscription(
            Detection,
            '/detection_topic',
            self.callback,
            10)
        self.ser = serial.Serial(
            port=port,
            baudrate=9600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )

    def callback(self, msg):
        self.get_logger().info("Received a /detection_topic message!")

        results = msg.results
        xywhn_list = msg.xywhn_list
        
        if len(results) > 0:
            if xywhn_list[0] > 0.75:
                command = 'right'
            elif xywhn_list[0] < 0.25:
                command = 'left'
            else:
                command = 'forward'
        else:
            command = 'stop'
        
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
