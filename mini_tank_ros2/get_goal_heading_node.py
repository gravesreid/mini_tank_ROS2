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
import numpy as np
import math



class Driver(Node):

    def __init__(self):
        super().__init__('yolo_listener')
        self.detection_subscription = self.create_subscription(
            Detection,
            '/detection_topic',
            self.detection_callback,
            10)
        self.compass_subscription = self.create_subscription(
            Float64,
            '/compass_degree',
            self.compass_callback,
            10)
        self.desired_heading_publisher = self.create_publisher(Float64, 'desired_heading', 10)

        self.publish_rate = 10.0
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.angle = 0
        self.compass_degree = 0



    def detection_callback(self, msg):
        self.get_logger().info("Received a /detection_topic message!")

        results = msg.results
        xywhn_list = msg.xywhn_list
        self.path_list = []
        self.waypoint_list = []

        if len(results) > 0:
            for i in range(len(results)):
                if results[i] == 0:
                    self.path_list.append(xywhn_list[(i*4):(4*(i+1))])
                elif results[i] == 1:
                    self.waypoint_list.append(xywhn_list[(i*4):(4*(i+1))])

        # get angle between the robot and the goal
        if self.path_list:
            self.angle = math.degrees(math.atan2(1 - self.path_list[0][1], self.path_list[0][0]))
        else:
            self.get_logger().info("No path detected")

    def compass_callback(self, msg):
        self.get_logger().info("Received a /compass_degree message!")
        self.compass_degree = msg.data

        # get the desired heading
        self.desired_heading = self.wrap_angle(self.angle - self.compass_degree)
        
    def wrap_angle(self, angle):
        return (angle + 180) % 360 - 180
    
    def timer_callback(self):
        if hasattr(self, 'desired_heading'):
            self.desired_heading_publisher.publish(Float64(data=self.desired_heading))
            del self.desired_heading
        




def main(args=None):
    rclpy.init(args=args)
    driver = Driver()
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
