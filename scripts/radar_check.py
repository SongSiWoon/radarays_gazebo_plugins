#!/usr/bin/env python3
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

rclpy.init()
node = rclpy.create_node('radar_check')
bridge = CvBridge()
count = 0

def cb(msg):
    global count
    img = bridge.imgmsg_to_cv2(msg, 'mono8')
    count += 1
    print(f'[{count}] Shape: {img.shape}, Min: {img.min()}, Max: {img.max()}, Mean: {img.mean():.2f}, NonZero: {np.count_nonzero(img)}')
    if count >= 5:
        rclpy.shutdown()

node.create_subscription(Image, '/radar/image', cb, 10)
rclpy.spin(node)
