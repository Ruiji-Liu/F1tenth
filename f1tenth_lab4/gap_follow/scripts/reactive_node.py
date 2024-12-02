#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: Subscribe to LIDAR
        # TODO: Publish to drive
        self.sub_lidar = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)
        
        self.window = 4
        self.threshold = 2.0
        self.disparity = 2.5
        self.rb = 0.2
        self.n = 10
        self.angle_increment = 0
        self.velocity = 5.0
    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        proc_ranges = ranges[134:-134]
        proc_ranges = np.array(proc_ranges)
        for i in range(0, len(proc_ranges), self.window):
            subset = ranges[i:i+self.window]
            mean = sum(subset)/len(subset)
            proc_ranges[i:i+self.window] = np.repeat(mean, self.window)
        return proc_ranges
    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
    
        
        largest_difference = None
        above_threshold_indices = np.where(free_space_ranges > self.threshold)[0]
        # print(above_threshold_indices)
        max_gap = [0, len(above_threshold_indices)-1]
        for i in range(1,len(above_threshold_indices)):
            if above_threshold_indices[i] - above_threshold_indices[i-1] != 1:
                max_gap.insert(-1,i)
        gap_depth = []
        for i in range(1,len(max_gap)):
            difference = max_gap[i] - max_gap[i - 1]
            if difference >= self.n:
                gap_depth.append(np.mean(free_space_ranges[above_threshold_indices[max_gap[i-1]]:above_threshold_indices[max_gap[i]-1]]))
            else:
                gap_depth.append(0)
        max_index = gap_depth.index(max(gap_depth))
        start_index = above_threshold_indices[max_gap[max_index]]
        end_index = above_threshold_indices[max_gap[max_index+1]-1]
        left_dis = abs(free_space_ranges[start_index] - free_space_ranges[start_index - 1])
        right_dis = abs(free_space_ranges[end_index + 1] - free_space_ranges[end_index])
        if left_dis > self.disparity and free_space_ranges[start_index-1]!= 0:
            print("swift to right")
            swifted_angle = self.rb/free_space_ranges[start_index-1]
            swifted_reading = swifted_angle/self.angle_increment
            start_index += int(swifted_reading)
            end_index += int(swifted_reading)
            # print("swifting reading", swifted_reading)
        if right_dis > self.disparity and free_space_ranges[end_index+1] != 0:
            print("swift to left")
            swifted_angle = self.rb/free_space_ranges[end_index+1]
            swifted_reading = swifted_angle/self.angle_increment
            start_index -= int(swifted_reading)
            end_index -= int(swifted_reading)

        return start_index, end_index


    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
        largest_gap = 0
        largest_index = 0
        for i in range(start_i, end_i):
            if largest_gap == 0 or ranges[i] > largest_gap:
                largest_gap = ranges[i]
                largest_index = i
        mid_point = (start_i + end_i) / 2
        steering_angle = (mid_point - (len(ranges)//2)) * self.angle_increment
        # print("largest gap:", largest_gap)
        # print("largest gap", largest_index)
        print("steering_angle",steering_angle)
        return steering_angle
    def create_safety_bubble(self,lidar_data, closest_distance, rb):
        safety_bubble_indices = np.where(lidar_data <= closest_distance + rb)
        print("safety",safety_bubble_indices)
        lidar_data[safety_bubble_indices] = 0

        return lidar_data
    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        self.angle_increment = data.angle_increment
        proc_ranges = self.preprocess_lidar(ranges)
        
        # TODO:
        #Find closest point to LiDAR
        closest_point_index = np.argmin(proc_ranges)
        print("cloest",closest_point_index)
        closest_point_distance = proc_ranges[closest_point_index]
        #Eliminate all points inside 'bubble' (set them to zero) 
        free_space_range = self.create_safety_bubble(proc_ranges, closest_point_distance, self.rb)
        start_i, end_i = self.find_max_gap(free_space_range)
        #Find the best point in the gap 
        steering_angle = self.find_best_point(start_i, end_i, ranges)

        #Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = self.velocity
        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()