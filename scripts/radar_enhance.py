#!/usr/bin/env python3
"""
Enhances radar polar image brightness for visualization.
Applies histogram equalization and contrast boost.

Subscribes: /radar/image
Publishes:  /radar/enhanced
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class RadarEnhance(Node):
    def __init__(self):
        super().__init__('radar_enhance')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/radar/image', self.callback, 10)
        self.pub_enhanced = self.create_publisher(Image, '/radar/enhanced', 10)
        self.pub_cart = self.create_publisher(Image, '/radar/cartesian', 10)
        self.get_logger().info('Radar enhance node started')

    def callback(self, msg):
        try:
            polar = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f'{e}')
            return

        n_range, n_angles = polar.shape
        self.get_logger().info(
            f'Image: {n_range}x{n_angles}, min={polar.min()}, max={polar.max()}, mean={polar.mean():.1f}',
            throttle_duration_sec=2.0)

        # Enhance: normalize to full 0-255 range
        enhanced = polar.copy()
        pmin, pmax = np.percentile(enhanced, [1, 99])
        if pmax > pmin:
            enhanced = np.clip((enhanced.astype(float) - pmin) / (pmax - pmin) * 255, 0, 255).astype(np.uint8)
        else:
            enhanced = cv2.equalizeHist(enhanced)

        # Publish enhanced polar
        out = self.bridge.cv2_to_imgmsg(enhanced, encoding='mono8')
        out.header = msg.header
        self.pub_enhanced.publish(out)

        # Also create cartesian bird's-eye view
        import math
        cart_size = 512
        cart = np.zeros((cart_size, cart_size), dtype=np.uint8)
        cx, cy = cart_size // 2, cart_size - 30

        min_angle = math.radians(-60.0)
        max_angle = math.radians(60.0)
        max_r = cart_size - 40

        for j in range(n_angles):
            angle = min_angle + (max_angle - min_angle) * j / max(n_angles - 1, 1)
            dx = math.sin(angle)
            dy = -math.cos(angle)

            step = max(1, n_range // max_r)
            for i in range(0, n_range, step):
                val = enhanced[i, j]
                if val < 10:
                    continue
                r = int(max_r * i / n_range)
                px = int(cx + r * dx)
                py = int(cy + r * dy)
                if 0 <= px < cart_size and 0 <= py < cart_size:
                    cart[py, px] = max(cart[py, px], val)

        # Draw robot and FOV
        cv2.circle(cart, (cx, cy), 4, 255, -1)
        for a in [min_angle, max_angle]:
            ex = int(cx + max_r * math.sin(a))
            ey = int(cy - max_r * math.cos(a))
            cv2.line(cart, (cx, cy), (ex, ey), 100, 1)

        cart_msg = self.bridge.cv2_to_imgmsg(cart, encoding='mono8')
        cart_msg.header = msg.header
        self.pub_cart.publish(cart_msg)


def main():
    rclpy.init()
    node = RadarEnhance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
