import rclpy                                    # Python Client Library for ROS 2
from rclpy.node import Node                     # Handles the creation of nodes
from sensor_msgs.msg import Image               # Image is the message type
import cv2                                      # OpenCV library
from cv_bridge import CvBridge                  # Package to convert between ROS and OpenCV Images
import numpy as np                              # Create the node and spin it



class ZedPublisher(Node):
    def __init__(self):
        super().__init__('zed_pub')
        self.publisher = self.create_publisher(Image, 'video_frames', 10)
        timer_period = 0.1  # seconds                                                            # FPS = 1 / timer_period
 
        self.timer = self.create_timer(timer_period, self.timer_callback)              # Create a timer with the callback function
        self.cap = cv2.VideoCapture(0)                                                  # Initialize the camera
        self.br = CvBridge()                                                            # Used to convert between ROS and OpenCV Images



    def timer_callback(self):                                                       # Check if there are subscribers
        ret, frame = self.cap.read()                                                # Read a frame from the camera
        frame = frame[0:480, 0:640]
        frame = cv2.resize(frame, (160, 120))

        if ret == True:
            # Publish the image.
            # The 'cv2_to_imgmsg' method converts an OpenCV
            # image to a ROS 2 image message
            self.publisher.publish(self.br.cv2_to_imgmsg(frame))

        # Display the message on the console
        self.get_logger().info('🤧 Publishing Zed frames 😮‍💨')




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