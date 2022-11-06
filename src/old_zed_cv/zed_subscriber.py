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


class ZEDSub(Node):
    """
    Subclass of ROS2 Node class. Receives and displays video feed from camera.
    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        super().__init__('zed_sub')
        self.callback_group = ReentrantCallbackGroup()
        self.subscription = self.create_subscription(CompressedImage, '/osiris/drive/zed/images',
                                                     self.receive_frame, qos_profile=qos_profile_sensor_data,
                                                     callback_group=self.callback_group)
        self.bridge = CvBridge()
        frame_width = 640
        frame_height = 360
        self.frame = np.zeros([frame_height, frame_width, 4], dtype=np.uint8)
        self.media_path = os.path.join(os.path.dirname(os.path.realpath(sys.argv[0])), 'science_recordings', 'zed2i')
        try:
            os.makedirs(self.media_path, exist_ok=True)
        except OSError as e:
            self.get_logger().info(f'{e}')
        now = datetime.now().strftime('%m-%d-%Y_%H:%M:%S')
        self.out = cv2.VideoWriter(os.path.join(self.media_path, 'ZED2i_'+now+'.mp4'),
                                   cv2.VideoWriter_fourcc(*'mp4v'), 30, (frame_width, frame_height))
        self.snapshot = False
        self.gui = False
        self.verbose = False

    def receive_frame(self, frame):
        """
        ROS2 subscriber callback. Receives frame via ROS2 topic. Saves image to mp4 if requested
        """
        self.frame = self.bridge.compressed_imgmsg_to_cv2(frame)
        self.out.write(self.frame)
        if self.verbose:
            self.get_logger().info('Receiving ZED frame')
        if self.gui:
            cv2.imshow('Drive View', self.frame)
            cv2.waitKey(1)
        if self.snapshot:
            now = datetime.now().strftime('%m-%d-%Y_%H:%M:%S')
            cv2.imwrite(os.path.join(self.media_path, 'ZED2i_' + now + '.jpeg'), self.frame)
            self.snapshot = False


def main(args=None):
    """
    Main function. Initializes and spins ROS2 node. Camera capture and node are ended explicitly before shutdown
    :param args: ROS2 command line arguments
    """
    rclpy.init(args=args)
    zed_sub = ZEDSub()
    zed_sub.gui = True
    cv2.namedWindow('Drive View', cv2.WINDOW_KEEPRATIO)
    try:
        rclpy.spin(zed_sub)
    except Exception as e:
        zed_sub.get_logger().info(f'{e})')
    except KeyboardInterrupt:
        print('\n')
    finally:
        if zed_sub.gui:
            cv2.destroyAllWindows()
        zed_sub.out.release()
        zed_sub.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

