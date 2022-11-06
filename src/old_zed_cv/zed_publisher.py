import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np


class Resolution:
    width = 1280
    height = 720


class ZEDPublisher(Node):
    """
    Subclass of ROS2 Node class. Accesses USB camera and publishes video feed
    """
    def __init__(self):
        """
        Class constructor.
        """
        super().__init__('zed_publisher')
        self.publisher = self.create_publisher(CompressedImage, '/osiris/drive/zed/images',
                                               qos_profile=qos_profile_sensor_data)
        timer_period = 1/30
        self.timer = self.create_timer(timer_period, self.publish_frame)
        self.camera = cv2.VideoCapture(0)
        self.bridge = CvBridge()
        image_size = Resolution()
        image_size.width = 1280
        image_size.height = 720
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, image_size.width * 2)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, image_size.height)

    def publish_frame(self):
        """
        Publisher callback function. Called every 0.01 seconds and publishes current frame to ROS2 topic.
        """
        if self.count_subscribers('/osiris/drive/zed/images') > 0:
            success, frame = self.camera.read()
            if success:
                frame = np.split(frame, 2, axis=1)[0]
                frame = cv2.resize(frame, (640, 360), interpolation=cv2.INTER_AREA)
                self.get_logger().info('Publishing ZED frame')
                self.publisher.publish(self.bridge.cv2_to_compressed_imgmsg(frame))
            else:
                self.get_logger().info(f'Unsuccessful frame capture')
        else:
            self.get_logger().info(f'No subscribers to receive message')


def main(args=None):
    """
    Main function. Initializes and spins ROS2 node. Camera capture and node are ended explicitly before shutdown
    :param args: ROS2 command line arguments
    """
    rclpy.init(args=args)
    zed_publisher = ZEDPublisher()
    try:
        rclpy.spin(zed_publisher)
    except Exception as e:
        zed_publisher.get_logger().info(f'{e})')
    except KeyboardInterrupt:
        print('\n')
    finally:
        zed_publisher.camera.release()
        zed_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

