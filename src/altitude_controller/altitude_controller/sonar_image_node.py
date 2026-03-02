#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
import cv2

from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge


class SonarImageNode(Node):

    def __init__(self):
        super().__init__('sonar_image_node')

        # ---- PARAMETERS (tune later) ----
        self.max_range = 40.0
        self.image_height = 512     # range bins
        self.image_width = 512      # bearings
        self.speckle_std = 0.15

        # ---- ROS ----
        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            LaserScan,
            'forward_looking_sonar/scan',
            self.scan_callback,
            10
        )

        self.pub = self.create_publisher(
            Image,
            '/sonar/image',
            10
        )

        self.get_logger().info("Sonar image node started")

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)

        # Replace inf / nan
        ranges[~np.isfinite(ranges)] = self.max_range

        num_beams = len(ranges)

        # Create empty sonar image
        sonar_img = np.zeros((self.image_height, num_beams), dtype=np.float32)

        # ---- IMAGE FORMATION ----
        for i, r in enumerate(ranges):

            r = np.clip(r, 0.3, self.max_range)

            # Convert range to pixel row
            row = int((r / self.max_range) * (self.image_height - 1))

            # Distance attenuation (1 / r^2)
            intensity = 1.0 / (r * r)

            # Beam spreading (cosine taper)
            angle = msg.angle_min + i * msg.angle_increment
            beam_gain = np.cos(angle) ** 2

            intensity *= beam_gain

            # Speckle noise (multiplicative)
            noise = np.random.normal(1.0, self.speckle_std)
            intensity *= noise

            sonar_img[row:, i] += intensity

        # ---- LOG COMPRESSION ----
        sonar_img = np.log1p(sonar_img)

        # Normalize to 8-bit
        sonar_img -= sonar_img.min()
        sonar_img /= sonar_img.max() + 1e-6
        sonar_img = (sonar_img * 255).astype(np.uint8)

        # Flip vertically (sonar convention)
        sonar_img = np.flipud(sonar_img)

        # Publish image
        img_msg = self.bridge.cv2_to_imgmsg(sonar_img, encoding='mono8')
        img_msg.header.stamp = msg.header.stamp
        img_msg.header.frame_id = msg.header.frame_id

        self.pub.publish(img_msg)


def main():
    rclpy.init()
    node = SonarImageNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
