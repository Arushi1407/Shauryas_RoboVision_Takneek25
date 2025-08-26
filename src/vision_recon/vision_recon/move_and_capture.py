#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import math
import time
import os

class CircleScanner(Node):
    def __init__(self):
        super().__init__('move_and_capture')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.capture_pub = self.create_publisher(Bool, '/capture_image', 10)

        self.radius = 1.0
        self.n_views = 8
        self.angular_speed = 0.5  # rad/s
        self.save_dir = os.path.expanduser('~/try/recon_images')
        os.makedirs(self.save_dir, exist_ok=True)

        self.scan_object()

    def scan_object(self):
        twist = Twist()
        angle_per_view = 2 * math.pi / self.n_views
        duration = angle_per_view / self.angular_speed

        for i in range(self.n_views):
            # Rotate around object
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed
            self.cmd_pub.publish(twist)
            self.get_logger().info(f"Rotating to capture view {i+1}/{self.n_views}")
            time.sleep(duration)

            # Stop
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            time.sleep(0.5)

            # Trigger image capture
            self.capture_pub.publish(Bool(data=True))
            self.get_logger().info(f"Captured image {i+1}")
            time.sleep(1.0)

def main(args=None):
    rclpy.init(args=args)
    node = CircleScanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
