#!/usr/bin/env python3

import numpy as np
from numpy import linalg as LA
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import OccupancyGrid

# TODO: import as you need
import random
from visualization_msgs.msg import Marker, MarkerArray
from tf_transformations import euler_from_quaternion
import csv
# class def for tree nodes
# It's up to you if you want to use this
# class Node(object):
#     def __init__(self):
#         self.x = None
#         self.y = None
#         self.parent = None
#         self.cost = None # only used in RRT*
#         self.is_root = False

# class def for RRT
class RRT(Node):
    def __init__(self):
        super().__init__('rrt_node')
        # topics, not saved as attributes
        # TODO: grab topics from param file, you'll need to change the yaml file
        pose_topic = "ego_racecar/odom"
        scan_topic = "/scan"

        # you could add your own parameters to the rrt_params.yaml file,
        # and get them here as class attributes as shown above.

        # TODO: create subscribers
        self.pose_sub_ = self.create_subscription(
            #PoseStamped,
            Odometry,
            pose_topic,
            self.pose_callback,
            1)
        self.pose_sub_

        self.scan_sub_ = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10)
        self.scan_sub_

        # self.goal_sub_ = self.create_subscription(
        #     #PoseStamped,
        #     Marker,
        #     '/waypoints_marker',
        #     self.goal_callback,
        #     10)
        # self.goal_sub_
        # publishers
        # TODO: create a drive message publisher, and other publishers that you might need

        # class attributes
        # TODO: maybe create your occupancy grid here
        self.occupancy_grid = None
        self.occupancy_grid_publisher = self.create_publisher(OccupancyGrid,'/occupancy_grid',10)
        self.tree_pub = self.create_publisher(MarkerArray, '/tree_marker', 10)
        self.path_pub = self.create_publisher(MarkerArray, '/path_marker', 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.waypoint_pub = self.create_publisher(Marker, '/waypoints_marker', 10)
        self.portion = 0.1
        self.goalx = 0
        self.goaly = 0
        self.x = 0
        self.y = 0
        self.lookahead = 3.0
        self.rrt_lookahead = 0.6
        self.vel = 0.6
        self.KP = 0.6
        self.i = 0
        self.collision_on_path = False
        self.read_waypoints('/home/ruiji_nb/sim_ws/src/f1tenth_lab5/pure_pursuit/scripts/new_waypoints.csv')
    def scan_callback(self, scan_msg):
        """
        LaserScan callback, you should update your occupancy grid here

        Args: 
            scan_msg (LaserScan): incoming message from subscribed topic
        Returns:

        """
        self.grid_size = 3.0  # Size of the grid (in meters)
        self.resolution = 0.1  # Resolution of the grid (in meters per cell)
        self.grid_width = int(self.grid_size / self.resolution)
        self.grid_height = int(self.grid_size / self.resolution)
        self.occupancy_grid = np.zeros((self.grid_width, self.grid_height), dtype=np.int8)

        ranges = scan_msg.ranges
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment

        for i, range_measurement in enumerate(ranges):
        # Skip invalid range measurements
            # if range_measurement == 0.0 or math.isinf(range_measurement):
            #     continue

            # Calculate angle for the current range measurement
            angle = - (((len(ranges)/2) - i) * angle_increment)

            # Convert polar coordinates to Cartesian coordinates
            x = range_measurement * math.cos(angle)
            y = range_measurement * math.sin(angle)
            # Convert Cartesian coordinates to grid coordinates
            grid_x = int((x ) / self.resolution) 
            grid_y = int((y + self.grid_size / 2.0)/ self.resolution)

        # Mark grid cell as occupied
            if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
                self.occupancy_grid[grid_y, grid_x] = 255  # Set the grid cell to occupied (1)
                # add thickness
                if grid_y <= self.grid_size/self.resolution-3 and grid_y >= 2 and grid_x <= self.grid_size/self.resolution-3 and grid_x >= 2:
                    for dy in range(-2, 3):
                        for dx in range(-2, 3):
                                self.occupancy_grid[grid_y + dy, grid_x + dx] = 255
        self.publish_occupancy_grid()
       
    def pose_callback(self, pose_msg):
        """
        The pose callback when subscribed to particle filter's inferred pose
        Here is where the main RRT loop happens

        Args: 
            pose_msg (PoseStamped): incoming message from subscribed topic
        Returns:

        """
        initial_x = pose_msg.pose.pose.position.x
        initial_y = pose_msg.pose.pose.position.y
        euler = euler_from_quaternion([
        pose_msg.pose.pose.orientation.x,
        pose_msg.pose.pose.orientation.y,
        pose_msg.pose.pose.orientation.z,
        pose_msg.pose.pose.orientation.w
    ])
        self.theta = euler[2]
        self.tree = [[initial_x,initial_y,None]]
        if self.occupancy_grid is not None:
            self.goalx, self.goaly, goal_index, pp_steering_angle = self.find_goal(self.waypoints, initial_x, initial_y)
            print("goal", self.goalx, self.goaly)
            self.marker_publisher(self.goalx, self.goaly)
            current_index = self.find_closest_index(initial_x, initial_y)
            reference_path = self.reference_path(current_index, goal_index)
            for i in range(1,len(reference_path)):
                local_waypoint1 = self.tranform_to_local(self.theta, reference_path[i-1], initial_x, initial_y)
                local_waypoint2 = self.tranform_to_local(self.theta, reference_path[i], initial_x, initial_y)
                if self.check_collision(local_waypoint1, local_waypoint2):
                    self.collision_on_path = True
                    break
                else:
                    self.collision_on_path = False
            # local_goal = self.tranform_to_local(self.theta, (self.goalx, self.goaly), initial_x, initial_y)
            # self.marker_publisher(local_goal[0], local_goal[1])
            print(self.collision_on_path)
            if self.collision_on_path is True: 
                print("Using Pure Pursuit with RRT")
                while self.is_goal(self.tree[-1], self.goalx, self.goaly) is False and len(self.tree) <= 400:
                    # print("here")
                    xrand = self.sample()
                    # print("xrand_local", xrand)
                    xrand = self.tranform_to_gloabl(self.theta, xrand, initial_x, initial_y)
                    # print("xrand_global", xrand)
                    nearest_node_index = self.nearest(xrand)
                    new_node = self.steer(self.tree[nearest_node_index], xrand)
                    local_nearest = self.tranform_to_local(self.theta, self.tree[nearest_node_index], initial_x, initial_y)
                    local_new = self.tranform_to_local(self.theta, new_node, initial_x, initial_y)
                    if self.check_collision(local_nearest, local_new) is True:
                        pass
                    else:
                        self.tree.append(new_node)
                    # print("tree",len(self.tree))
                    # self.i += 1
                    # print("times:", self.i)
                if self.is_goal(self.tree[-1], self.goalx, self.goaly):
                    print("find goal")
                    self.path = self.find_path(self.tree, self.tree[-1])
                    # print("path", self.tree)
                    self.publish_path(self.path, color=(0.0, 1.0, 0.0) )
                    self.publish_tree(self.tree, color=(1.0, 0.0, 0.0) )
                    self.pure_pursuit(initial_x, initial_y)
            else:
                print("Using Pure Pursuit Only")
                drive_msg = AckermannDriveStamped()
                drive_msg.drive.steering_angle = pp_steering_angle * self.KP
                drive_msg.drive.speed = self.vel
                self.drive_pub.publish(drive_msg)


        # print("tree", self.tree)
    # test_node1 = [3.3477296399227883,-1.1042110324846803,None]
            # test_node2 = [3.2113748850215504,-0.9903227086977804,None]
            # local_nearest = self.tranform_to_local(self.theta, test_node1, initial_x, initial_y)
            # local_new = self.tranform_to_local(self.theta, test_node2, initial_x, initial_y)
            # print("collision detected",self.check_collision(local_nearest, local_new))
            # test_point = (1,0)
            # global_point = self.tranform_to_gloabl(self.theta, test_point, initial_x, initial_y)
            # print("global_point", global_point)
            # print("xrand", xrand[0])
            # test_node1 = [2,0,None]
            # test_node2 = [2,-1.5,None]
            # print("collision detected",self.check_collision(test_node1, test_node2))
            # print("Is goal", self.is_goal(test_node1, 2, 0.09))
    

    def pure_pursuit(self, current_x, current_y):
        min_dist = float('inf')
        for i in range(len(self.path)):
            distance = np.sqrt((current_x - float(self.path[i][0]))**2 + (current_y - float(self.path[i][1]))**2)
            if self.rrt_lookahead < distance < min_dist:
                x_v = (float(self.path[i][0]) - current_x) * np.cos(self.theta) + (float(self.path[i][1]) - current_y) * np.sin(self.theta)
                y_v = -(float(self.path[i][0]) - current_x) * np.sin(self.theta) + (float(self.path[i][1]) - current_y) * np.cos(self.theta)
                angle = np.arctan2(y_v, x_v)
                if abs(angle) < 2* np.pi / 3:
                    min_dist = distance
                    steering_angle = (2 * y_v) / distance**2 
        # TODO: transform goal point to vehicle frame of reference
        
        # TODO: publish drive message, don't forget to limit the steering angle.
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle * self.KP
        print("steering angle", steering_angle)
        drive_msg.drive.speed = self.vel
        self.drive_pub.publish(drive_msg)

    def sample(self):
        """
        This method should randomly sample the free space, and returns a viable point

        Args:
        Returns:
            (x, y) (float float): a tuple representing the sampled point

        """
        while True:
        # Generate random grid coordinates within the bounds of the occupancy grid
            grid_x = random.randint(0, self.grid_width - 1)
            grid_y = random.randint(0, self.grid_height - 1)

            if self.occupancy_grid[grid_x, grid_y] == 0:
                x = grid_y * self.resolution 
                y = (grid_x * self.resolution - self.grid_size / 2.0)
                # self.publish_marker((x, y))  
                return (x, y)
    
    def nearest(self, sampled_point):
        """
        This method should return the nearest node on the tree to the sampled point

        Args:
            tree ([]): the current RRT tree
            sampled_point (tuple of (float, float)): point sampled in free space
        Returns:
            nearest_node (int): index of neareset node on the tree
        """
        min_dist = float('inf')
        nearest_node_index = None

        for i, node in enumerate(self.tree):
            dist = ((sampled_point[0] - node[0]) ** 2 + (sampled_point[1] - node[1]) ** 2) ** 0.5
            if dist < min_dist:
                min_dist = dist
                nearest_node_index = i

        return nearest_node_index

    def steer(self, nearest_node, sampled_point):
        """
        This method should return a point in the viable set such that it is closer 
        to the nearest_node than sampled_point is.

        Args:
            nearest_node (Node): nearest node on the tree to the sampled point
            sampled_point (tuple of (float, float)): sampled point
        Returns:
            new_node (Node): new node created from steering
        """
        
        dx = sampled_point[0] - nearest_node[0]
        dy = sampled_point[1] - nearest_node[1]

        new_node_x = nearest_node[0] + dx * self.portion
        new_node_y = nearest_node[1] + dy * self.portion

        nearest_node_index = self.tree.index(nearest_node)
        new_node = [new_node_x, new_node_y, nearest_node_index]
        return new_node

    def check_collision(self, nearest_node, new_node):
        """
        This method should return whether the path between nearest and new_node is
        collision free.

        Args:
            nearest (Node): nearest node on the tree
            new_node (Node): new node from steering
        Returns:
            collision (bool): whether the path between the two nodes are in collision
                              with the occupancy grid
        """
        x1 = nearest_node[0]
        y1 = nearest_node[1]
        x2 = new_node[0]
        y2 = new_node[1]

        new_grid_x = int((x2 ) / self.resolution) 
        new_grid_y = int((y2 + self.grid_size / 2.0)/ self.resolution)
        if self.occupancy_grid[new_grid_y, new_grid_x] != 0:
            return True
        else:
            distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            num_steps = int(distance / self.resolution)
            for i in range(num_steps):
                # Interpolate coordinates along the path
                x = x1 + (x2 - x1) * i / num_steps
                y = y1 + (y2 - y1) * i / num_steps
                
                # Convert coordinates to grid indices
                grid_x = int((x ) / self.resolution) 
                grid_y = int((y + self.grid_size / 2.0)/ self.resolution)
                
                # print(self.occupancy_grid[grid_y,grid_x])
                # Check if the grid cell is occupied
                if self.occupancy_grid[grid_y, grid_x] != 0:
                    # print("grid x", grid_x)
                    # print("grid y", grid_y)
                    return True  # Collision detected
            return False

    def is_goal(self, latest_added_node, goal_x, goal_y):
        """
        This method should return whether the latest added node is close enough
        to the goal.

        Args:
            latest_added_node (Node): latest added node on the tree
            goal_x (double): x coordinate of the current goal
            goal_y (double): y coordinate of the current goal
        Returns:
            close_enough (bool): true if node is close enoughg to the goal
        """
        latest_x = latest_added_node[0]
        latest_y = latest_added_node[1]
        distance_to_goal = math.sqrt((latest_x - goal_x) ** 2 + (latest_y - goal_y) ** 2)
        threshold = 0.1
        close_enough = distance_to_goal < threshold
        return close_enough

    def find_path(self, tree, latest_added_node):
        """
        This method returns a path as a list of Nodes connecting the starting point to
        the goal once the latest added node is close enough to the goal

        Args:
            tree ([]): current tree as a list of Nodes
            latest_added_node (Node): latest added node in the tree
        Returns:
            path ([]): valid path as a list of Nodes
        """
        path = []
        current_node = latest_added_node
        while current_node[2] is not None:
        # Insert the current node at the beginning of the path list
            path.insert(0, current_node)
            # print(current_node)
            # Update the current node to its parent node
            current_node = tree[current_node[2]]
        path.insert(0, self.tree[0])
        return path
    def find_goal(self, waypoints, initial_x, initial_y):
        min_dist = float('inf')
        min_index = 0
        for i in range(len(self.waypoints)):
            distance = np.sqrt((initial_x - float(self.waypoints[i][0]))**2 + (initial_y - float(self.waypoints[i][1]))**2)
            if self.lookahead < distance < min_dist:
                x_v = (float(self.waypoints[i][0]) - initial_x) * np.cos(self.theta) + (float(self.waypoints[i][1]) - initial_y) * np.sin(self.theta)
                y_v = -(float(self.waypoints[i][0]) - initial_x) * np.sin(self.theta) + (float(self.waypoints[i][1]) - initial_y) * np.cos(self.theta)
                angle = np.arctan2(y_v, x_v)
                if abs(angle) < 2* np.pi / 3:
                    min_dist = distance
                    min_index = i
                    pp_steering_angle = (2 * y_v) / distance**2
        # return float(waypoints[min_index][0]), float(waypoints[min_index][1])
        # print("min index", min_index)
        return waypoints[min_index][0], waypoints[min_index][1], min_index, pp_steering_angle
    def find_closest_index(self, initial_x, initial_y):
        # print("current xy", initial_x, initial_y)
        min_dist = float('inf')
        min_index = 0
        for i in range(len(self.waypoints)):
            distance = np.sqrt((initial_x - float(self.waypoints[i][0]))**2 + (initial_y - float(self.waypoints[i][1]))**2)
            if distance < min_dist:
                min_dist = distance
                min_index = i
        return min_index
    def reference_path(self, current_index, goal_index):
        if current_index <= goal_index:
            return self.waypoints[current_index:goal_index+1] 
        else:
            return self.waypoints[current_index:] + self.waypoints[:goal_index+1]
    def read_waypoints(self,file_path):
        waypoints = []
        with open(file_path, mode='r') as csv_file:
            csv_reader = csv.reader(csv_file)
            for row in csv_reader:
                row = [float(element) for element in row]
                waypoints.append(row)
        # print(waypoints)
        self.waypoints = waypoints
    def goal_callback(self, marker_msg):
        self.goalx = marker_msg.pose.position.x
        self.goaly = marker_msg.pose.position.y

        # print("goal", self.goalx, self.goaly)

        # return(goal_x, goal_y)
    def tranform_to_gloabl(self, theta, new_point, robot_pos_x, robot_pos_y):
        x_r = new_point[0]
        y_r = new_point[1]
        x_g = x_r * math.cos(theta) - y_r * math.sin(theta) + robot_pos_x
        y_g = x_r * math.sin(theta) + y_r * math.cos(theta) + robot_pos_y
        return (x_g, y_g)
    
    def tranform_to_local(self, theta, global_point, robot_pos_x, robot_pos_y):
        dx = global_point[0] - robot_pos_x
        dy = global_point[1] - robot_pos_y

        # Rotate the difference vector by -robot_yaw to align with the local frame
        local_x = dx * math.cos(-theta) - dy * math.sin(-theta)
        local_y = dx * math.sin(-theta) + dy * math.cos(-theta)

        return (local_x, local_y)

    # The following methods are needed for RRT* and not RRT
    def cost(self, tree, node):
        """
        This method should return the cost of a node

        Args:
            node (Node): the current node the cost is calculated for
        Returns:
            cost (float): the cost value of the node
        """
        return 0

    def line_cost(self, n1, n2):
        """
        This method should return the cost of the straight line between n1 and n2

        Args:
            n1 (Node): node at one end of the straight line
            n2 (Node): node at the other end of the straint line
        Returns:
            cost (float): the cost value of the line
        """
        return 0

    def near(self, tree, node):
        """
        This method should return the neighborhood of nodes around the given node

        Args:
            tree ([]): current tree as a list of Nodes
            node (Node): current node we're finding neighbors for
        Returns:
            neighborhood ([]): neighborhood of nodes as a list of Nodes
        """
        neighborhood = []
        return neighborhood
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

    def publish_path(self, path, color):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = 'map'  # Assuming the frame ID is 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # Point size
        marker.scale.y = 0.1
        marker.color.a = 1.0  # Alpha (transparency)
        marker.color.r, marker.color.g, marker.color.b = color
        marker.points = [Point(x=p[0], y=p[1], z=0.0) for p in path]  # Convert points to Point messages
        marker_array.markers.append(marker)
        self.path_pub.publish(marker_array)
    def publish_tree(self, tree, color):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = 'map'  # Assuming the frame ID is 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # Point size
        marker.scale.y = 0.1
        marker.color.a = 1.0  # Alpha (transparency)
        marker.color.r, marker.color.g, marker.color.b = color
        marker.points = [Point(x=p[0], y=p[1], z=0.0) for p in tree]  # Convert points to Point messages
        marker_array.markers.append(marker)
        self.tree_pub.publish(marker_array)
    def publish_occupancy_grid(self):
        # Create OccupancyGrid message
        occupancy_grid_msg = OccupancyGrid()
        occupancy_grid_msg.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid_msg.header.frame_id = 'map'
        occupancy_grid_msg.info.width = self.grid_width
        occupancy_grid_msg.info.height = self.grid_height
        occupancy_grid_msg.info.resolution = self.resolution
        occupancy_grid_msg.info.origin.position.x = 0.0
        occupancy_grid_msg.info.origin.position.y = -self.grid_size / 2.0
        occupancy_grid_msg.info.origin.position.z = 0.0
        occupancy_grid_msg.info.origin.orientation.x = 0.0
        occupancy_grid_msg.info.origin.orientation.y = 0.0
        occupancy_grid_msg.info.origin.orientation.z = 0.0
        occupancy_grid_msg.info.origin.orientation.w = 1.0
        occupancy_grid_msg.data = np.ravel(self.occupancy_grid).tolist()
        # print(np.ravel(self.occupancy_grid).tolist())
        # Publish the occupancy grid
        self.occupancy_grid_publisher.publish(occupancy_grid_msg)
def main(args=None):
    rclpy.init(args=args)
    print("RRT Initialized")
    rrt_node = RRT()
    rclpy.spin(rrt_node)

    rrt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
