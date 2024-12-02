#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive

class AckermannDrivePublisher(Node):
    def __init__(self):
        super().__init__('ackermann_drive_publisher')
        self.publisher_ = self.create_publisher(AckermannDrive, '/ackermann_drive', 10)
        self.timer_ = self.create_timer(1.0, self.publish_ackermann_drive)
        self.speed_ = 1.0  # adjust the speed as needed

    def publish_ackermann_drive(self):
        ackermann_msg = AckermannDrive()
        ackermann_msg.speed = self.speed_
        self.publisher_.publish(ackermann_msg)
        self.get_logger().info(f'Published AckermannDrive: speed={self.speed_}')

def main(args=None):
    rclpy.init(args=args)
    ackermann_drive_publisher = AckermannDrivePublisher()
    rclpy.spin(ackermann_drive_publisher)
    ackermann_drive_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()