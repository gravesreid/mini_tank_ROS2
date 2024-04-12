#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import struct
import sys
import time
import math
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist
from bittle_msgs.msg import Detection

class Controller(Node):

    def __init__(self, port='/dev/ttyUSB0'):
        super().__init__('mag_controller_node')
        self.subscription = self.create_subscription(
            Float64,
            '/compass_degree',
            self.compass_callback,
            10)
        self.subscription = self.create_subscription(
            Float64,
            '/desired_heading',
            self.desired_heading_callback,
            10)
        self.control_publisher = self.create_publisher(Float64, 'control_output', 10)
        self.ser = serial.Serial(
            port=port,
            baudrate=9600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )

        self.compass_degree = 0
        self.desired_heading = 1000

        # PID parameters
        self.kp = 10
        self.ki = 0
        self.kd = 0
        self.integral = 0
        self.prev_error = 0
        self.integral_max = 100/ self.ki if self.ki > 0 else 0

    def compass_callback(self, msg):
        self.compass_degree = msg.data
        self.get_logger().info("Received a /compass_degree message!")

    def desired_heading_callback(self, msg):
        self.desired_heading = msg.data
        self.get_logger().info("Received a /desired_heading message!")

    def serialWrite(self, command):
        if self.desired_angle == 1000:
            self.ser.write('0,0\n')
        else:
            self.ser.write((command + '\n').encode())

    def pid_controller(self):
        error = self.desired_heading - self.compass_degree

        error = (error + 180) % 360 - 180 

        self.integral += error
        self.integral = max(-self.integral_max, min(self.integral_max, self.integral))

        derivative = error - self.prev_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error

        self.serialWrite(self.calculate_motor_commands(output))
        self.control_publisher.publish(Float64(data=output))

    
    def calculate_motor_commands(self, pid_output):
        motor_left = max(min(255, 255 - pid_output), -255)
        motor_right = max(min(255, 240 + pid_output), -255)
        return f'{motor_left} {motor_right}'


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()