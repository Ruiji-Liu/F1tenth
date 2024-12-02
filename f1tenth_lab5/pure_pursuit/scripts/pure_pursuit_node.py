#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import csv
from tf_transformations import euler_from_quaternion
# TODO CHECK: include needed ROS msg type headers and libraries

class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')
        # TODO: create ROS subscribers and publishers
        drive_topic = '/drive'
        odom_topic = '/ego_racecar/odom'

        self.waypoint_pub = self.create_publisher(
            Marker, '/waypoints_marker', 10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, drive_topic, 10)
        self.sub_odom = self.create_subscription(
            Odometry, odom_topic, self.pose_callback,10)
        
        self.lookahead = 1.2
        self.vel = 1.5
        self.x = 0
        self.y = 0
        self.theta = 0
        self.read_waypoints('/home/ruiji_nb/sim_ws/src/f1tenth_lab5/pure_pursuit/scripts/new_waypoints.csv')
        # self.read_waypoints('/home/ruiji_nb/sim_ws/src/f1tenth_lab5/pure_pursuit/scripts/waypoint1.csv')
    def pose_callback(self, pose_msg):
        self.x = pose_msg.pose.pose.position.x
        self.y = pose_msg.pose.pose.position.y
        euler = euler_from_quaternion([
        pose_msg.pose.pose.orientation.x,
        pose_msg.pose.pose.orientation.y,
        pose_msg.pose.pose.orientation.z,
        pose_msg.pose.pose.orientation.w
    ])
        self.theta = euler[2]
        # TODO: find the current waypoint to track using methods mentioned in lecture
        min_dist = float('inf')
        min_index = 0
        for i in range(len(self.waypoints)):
            distance = np.sqrt((self.x - float(self.waypoints[i][0]))**2 + (self.y - float(self.waypoints[i][1]))**2)
            if self.lookahead < distance < min_dist:
                x_v = (float(self.waypoints[i][0]) - self.x) * np.cos(self.theta) + (float(self.waypoints[i][1]) - self.y) * np.sin(self.theta)
                y_v = -(float(self.waypoints[i][0]) - self.x) * np.sin(self.theta) + (float(self.waypoints[i][1]) - self.y) * np.cos(self.theta)
                angle = np.arctan2(y_v, x_v)
                if abs(angle) < 2* np.pi / 3:
                    min_dist = distance
                    min_index = i
                    steering_angle = (2 * y_v) / distance**2
        # TODO: transform goal point to vehicle frame of reference
        
        # TODO: publish drive message, don't forget to limit the steering angle.
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = self.vel
        # self.drive_pub.publish(drive_msg)

        #waypoint marker
        next_x = float(self.waypoints[min_index][0])
        next_y = float(self.waypoints[min_index][1])
        self.marker_publisher(next_x, next_y)
    
    def read_waypoints(self,file_path):
        waypoints = []
        with open(file_path, mode='r') as csv_file:
            csv_reader = csv.reader(csv_file)
            for row in csv_reader:
                waypoints.append(row)
        self.waypoints = waypoints
    def marker_publisher(self, x, y):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0  # Fully opaque
        marker.color.r = 1.0  # Red
        marker.pose.orientation.w = 1.0  # Default orientation

        # Set the marker position
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.lifetime = rclpy.duration.Duration(seconds=1).to_msg()
        self.waypoint_pub.publish(marker)
def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
