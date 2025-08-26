#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import os
import time

class CameraSaver(Node):
    def __init__(self):
        super().__init__('camera_saver')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('out_dir', os.path.expanduser('~/try/recon_images'))

        image_topic = self.get_parameter('image_topic').value
        self.out_dir = self.get_parameter('out_dir').value
        os.makedirs(self.out_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.trigger_sub = self.create_subscription(Bool, '/capture_image', self.trigger_callback, 10)

        self.last_image = None
        self.counter = 0

    def image_callback(self, msg):
        self.last_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def trigger_callback(self, msg):
        if msg.data and self.last_image is not None:
            filename = os.path.join(self.out_dir, f'image_{self.counter:03d}.png')
            cv2.imwrite(filename, self.last_image)
            self.get_logger().info(f"Saved {filename}")
            self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = CameraSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
