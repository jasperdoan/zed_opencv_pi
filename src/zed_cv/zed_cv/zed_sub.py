import rclpy                        # Python library for ROS 2
from rclpy.node import Node         # Handles the creation of nodes
from sensor_msgs.msg import Image   # Image is the message type
from cv_bridge import CvBridge      # Package to convert between ROS and OpenCV Images
import cv2                          # OpenCV library
 
class ZedSubscriber(Node):
  def __init__(self):
    super().__init__('zed_sub')
    self.subscription = self.create_subscription(
      Image, 
      'video_frames', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning
      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
   
  def listener_callback(self, data):
    # Display the message on the console
    self.get_logger().info('😩 Receiving video frame!! 😫')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)
    
    # Display image
    cv2.imshow("Zed view", current_frame)
    
    cv2.waitKey(1)
  


def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  zed_sub = ZedSubscriber()
  
  # Spin the node so the callback function is called.
  rclpy.spin(zed_sub)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  zed_sub.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()