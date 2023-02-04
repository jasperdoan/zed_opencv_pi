import rclpy                                                # Python library for ROS 2
from rclpy.node import Node                                 # Handles the creation of nodes
from sensor_msgs.msg import Image                           # Image is the message type
import cv2                                                  # OpenCV library
from cv_bridge import CvBridge                              # Package to convert between ROS and OpenCV Images
import numpy as np                                          # Python library for scientific computing



class ZedSubscriber(Node):
    def __init__(self):
        super().__init__('zed_sub')                                                                             # Create the node                                                         # Create a callback group

        # Create a subscriber with the appropriate topic, message type, and callback function
        self.subscription = self.create_subscription( Image, 'video_frames', self.listener_callback, 10)
        self.subscription                                                                                       # Prevent unused variable warning   
        
        self.br = CvBridge()                                                                                    # Used to convert between ROS and OpenCV images
        # frame_width = 640                                                                                       # Set the frame width
        # frame_height = 360                                                                                      # Set the frame height

        # self.frame = np.zeros([frame_height, frame_width, 4], dtype=np.uint8)                                   # Create a blank image
        # self.media_path = os.path.join(os.path.dirname(os.path.realpath(sys.argv[0])), 'recordings', 'zed2i')   # Set the path to save the video

        # try:                                                                                                    # Try to create the directory   
        #     os.makedirs(self.media_path, exist_ok=True)                                                         # Create the directory if it does not exist   
        # except OSError as e:                                                                                    # Catch any error that occurs
        #     self.get_logger().exception("Failed to create media parent directory")                              # Print the error

        # now = datetime.now().strftime('%m-%d-%Y_%H:%M:%S')                                                      # Get the current date and time
        # self.out = cv2.VideoWriter(os.path.join(self.media_path, 'ZED2i_'+now+'.mp4'),                          # Create the video writer
        #                            cv2.VideoWriter_fourcc(*'mp4v'), 30, (frame_width, frame_height))            # Set the video codec, frame rate, and frame size

        self.get_logger().info('ZED Subscriber has been initialized')                                           # Print status message
                                                                                          # Set the verbose flag to false
     


    def listener_callback(self, data):
        self.get_logger().info('😩 Receiving video frame!! 😫')                                             # Display the message on the console
        
        current_frame = self.br.imgmsg_to_cv2(data)
        cv2.imshow("camera", current_frame)

        cv2.waitKey(1)




def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_subscriber = ZedSubscriber()

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



# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
  
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
 
class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
      'video_frames', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning
      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
   
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)
    
    # Display image
    cv2.imshow("camera", current_frame)
    
    cv2.waitKey(1)
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = ImageSubscriber()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()