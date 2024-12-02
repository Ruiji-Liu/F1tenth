#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from rclpy.time import Time

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        # Subscribe to LaserScan and Odometry
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/ego_racecar/odom', self.odom_callback, 10)

        # Create a publisher for AckermannDriveStamped
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)

        # Initialize variables
        self.iTTC_threshold = 0.8  # Adjust this threshold as needed
        self.prev_range_measurement = None
        self.prev_timestamp = None
        if self.prev_range_measurement is None:
            print("once")
            self.initial_speed()

    def laser_callback(self, msg):
        # Calculate iTTC based on LaserScan data
        
        iTTC_array = self.calculate_iTTC(msg)
        # if iTTC_array is None:
        #     pass
        # print("minimum iTTC", np.min(iTTC_array))
        # Analyze iTTC array and decide whether to brake
        
        if iTTC_array is not None:
            if self.collision_imminent(iTTC_array):
                # Publish AckermannDriveStamped with speed set to 0.0 m/s
                self.publish_brake_command()

    def odom_callback(self, msg):
        # Update vehicle's velocity or other relevant information
        pass

    def calculate_iTTC(self, laser_msg):
        # Implement the calculation of iTTC from LaserScan data
        # ...
        r = np.array(laser_msg.ranges)
        # current_timestamp = laser_msg.header.stamp
        current_timestamp = Time(seconds=laser_msg.header.stamp.sec, nanoseconds=laser_msg.header.stamp.nanosec)
        # print(current_timestamp)
        if self.prev_timestamp is not None and self.prev_range_measurement is not None:
            time_difference = current_timestamp - self.prev_timestamp
            time_difference = time_difference.nanoseconds

            range_difference = -(self.prev_range_measurement - r)
            rdot = range_difference / (time_difference / 1e9)
            rdot = np.maximum(-rdot, 0)
            return  r / rdot
        self.prev_timestamp = current_timestamp
        self.prev_range_measurement = r
        

    def collision_imminent(self, iTTC_array):
        # Analyze iTTC array and decide if collision is imminent
        # Apply threshold and handle false positives
        return any(iTTC < self.iTTC_threshold for iTTC in iTTC_array)
    def initial_speed(self):
        initial = AckermannDriveStamped()
        initial.drive.speed = 0.5
        initial.drive.steering_angle = 0.2
        self.drive_pub.publish(initial)

    def publish_brake_command(self):
        # Publish AckermannDriveStamped with speed set to 0.0 m/s
        brake_command = AckermannDriveStamped()
        brake_command.drive.speed = 0.0
        self.drive_pub.publish(brake_command)
        


def main():
    rclpy.init()
    safety_node = SafetyNode()
    rclpy.spin(safety_node)
    safety_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
