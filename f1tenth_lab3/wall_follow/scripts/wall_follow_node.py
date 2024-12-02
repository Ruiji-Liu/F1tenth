#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: create subscribers and publishers
        self.sub_lidar = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)
        # TODO: set PID gains
        self.kp = 1.0
        self.kd = 0.3
        self.ki = 0.0

        # TODO: store history
        self.integral = 0
        self.prev_error = 0 
        self.error = 0

        # TODO: store any necessary values you think you'll need
        self.angle_increment = 0
        self.theta = np.pi/4
        self.des_distance = 0.8
        self.lookahead_distance = 0.0
        self.des_vel = 2
        

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """

        #TODO: implement
        return range_data[int(angle//self.angle_increment + len(range_data) // 2)]

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """

        #TODO:implement
        # following the left
        a = self.get_range(range_data, np.pi/2 - self.theta)
        b = self.get_range(range_data, np.pi/2)
        alpha = -np.arctan2(a*np.cos(self.theta) - b, a*np.sin(self.theta))
        Dt = b * np.cos(alpha)
        Dt1 = Dt + self.lookahead_distance*np.sin(alpha)
        et = dist - Dt1
        return et, alpha

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        angle = 0.0
        steering_angle = -(self.kp * error + self.kd * (self.prev_error - error))
        print("steering angle is:", steering_angle)
        # TODO: Use kp, ki & kd to implement a PID controller
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle
        if steering_angle <= 0.174533:
            velocity = 1.5
        elif 0.174533 < steering_angle <= 0.349066:
            velocity = 1.0
        else:
            velocity = 0.5
        # TODO: fill in drive message and publish
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)
    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        # error = 0.0 # TODO: replace with error calculated by get_error()
        # velocity = 0.0 # TODO: calculate desired car velocity based on error
        ranges = np.array(msg.ranges)
        self.angle_increment = msg.angle_increment
        error, alpha = self.get_error(ranges, self.des_distance)
        self.pid_control(error, self.des_vel)
        self.prev_error = alpha
        # self.pid_control(error, self.des_vel) # TODO: actuate the car with PID


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()