import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import os
import sys
from datetime import datetime
import numpy as np


class ZedSubscriber(Node):
    def __init__(self):
        super().__init__('zed_sub')                                                                             # Create the node                                                         # Create a callback group

        self.callback_group = ReentrantCallbackGroup()
        self.subscription = self.create_subscription(CompressedImage, 'images',
                                                     self.listener_callback, qos_profile=qos_profile_sensor_data, callback_group=self.callback_group)   
        
        self.bridge = CvBridge()
        frame_width = 640
        frame_height = 360
        self.frame = np.zeros([frame_height, frame_width, 4], dtype=np.uint8)

        self.gui = False
        self.verbose = False



    def listener_callback(self, data):
        self.frame = self.bridge.compressed_imgmsg_to_cv2(data)

        if self.verbose:
            self.get_logger().info('Receiving ZED frame')
        if self.gui:
            cv2.imshow('Drive View', self.frame)
            cv2.waitKey(1)




def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_subscriber = ZedSubscriber()
    image_subscriber.gui = True
    cv2.namedWindow('Drive View', cv2.WINDOW_KEEPRATIO)

    # Spin the node so the callback function is called.
    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()                                         # Shutdown the rclpy library



if __name__ == '__main__':
    main()