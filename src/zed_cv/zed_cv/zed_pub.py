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


class ZedPublisher(Node):
    def __init__(self):
        super().__init__('zed_pub')
        self.publisher = self.create_publisher(CompressedImage, 'images',
                                               qos_profile=qos_profile_sensor_data)
        timer_period = 1/30
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.camera = cv2.VideoCapture(0)
        self.bridge = CvBridge()
        image_size = Resolution()
        image_size.width = 1280
        image_size.height = 720
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, image_size.width * 2)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, image_size.height)



    def timer_callback(self):                                                       # Check if there are subscribers
        success, frame = self.camera.read()
        if success:
            frame = np.split(frame, 2, axis=1)[0]
            frame = cv2.resize(frame, (640, 360), interpolation=cv2.INTER_AREA)
            self.get_logger().info('Publishing ZED frame')
            self.publisher.publish(self.bridge.cv2_to_compressed_imgmsg(frame))
        else:
            self.get_logger().info(f'Unsuccessful frame capture')




def main(args=None):
  
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_publisher = ZedPublisher()

    # Spin the node so the callback function is called.
    rclpy.spin(image_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_publisher.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()