#!/usr/bin/env python3
"""
Converts RadaRays polar radar image to a cartesian (bird's-eye) view
for intuitive visualization.

Subscribes: /radar/image  (sensor_msgs/Image, mono8, polar format)
Publishes:  /radar/cartesian (sensor_msgs/Image, mono8, bird's-eye view)
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class RadarPolarToCart(Node):
    def __init__(self):
        super().__init__('radar_polar_to_cart')

        # Radar parameters (match URDF settings)
        self.declare_parameter('min_angle', -60.0)  # degrees
        self.declare_parameter('max_angle', 60.0)    # degrees - computed from min_angle + increment * samples
        self.declare_parameter('cart_size', 512)      # output image size (square)

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/radar/image', self.callback, 10)
        self.pub = self.create_publisher(Image, '/radar/cartesian', 10)

        self.get_logger().info('Radar polar-to-cartesian converter started')

    def callback(self, msg):
        try:
            polar = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        n_range_bins, n_angles = polar.shape
        cart_size = self.get_parameter('cart_size').value

        min_angle_deg = self.get_parameter('min_angle').value
        # Compute max angle from polar image dimensions
        # Each column = one angle sample
        increment_deg = (abs(min_angle_deg) * 2.0) / max(n_angles - 1, 1)
        max_angle_deg = min_angle_deg + increment_deg * (n_angles - 1)

        min_angle = math.radians(min_angle_deg)
        max_angle = math.radians(max_angle_deg)

        # Create cartesian image (bird's-eye view)
        cart = np.zeros((cart_size, cart_size), dtype=np.uint8)

        cx = cart_size // 2  # robot at center-bottom
        cy = cart_size - 20  # slight offset from bottom

        max_radius = cart_size - 30  # pixels for max range

        for j in range(n_angles):
            angle = min_angle + (max_angle - min_angle) * j / max(n_angles - 1, 1)
            # Radar forward = +x, left = +y
            # Image: up = -y_img, right = +x_img
            dx = math.sin(angle)
            dy = -math.cos(angle)

            for i in range(0, n_range_bins, max(1, n_range_bins // max_radius)):
                val = polar[i, j]
                if val < 5:
                    continue

                r = int(max_radius * i / n_range_bins)
                px = int(cx + r * dx)
                py = int(cy + r * dy)

                if 0 <= px < cart_size and 0 <= py < cart_size:
                    cart[py, px] = max(cart[py, px], val)

        # Draw robot position
        cv2.circle(cart, (cx, cy), 3, 255, -1)

        # Add FOV lines
        for angle in [min_angle, max_angle]:
            ex = int(cx + max_radius * math.sin(angle))
            ey = int(cy - max_radius * math.cos(angle))
            cv2.line(cart, (cx, cy), (ex, ey), 80, 1)

        # Publish
        out_msg = self.bridge.cv2_to_imgmsg(cart, encoding='mono8')
        out_msg.header = msg.header
        self.pub.publish(out_msg)


def main():
    rclpy.init()
    node = RadarPolarToCart()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
