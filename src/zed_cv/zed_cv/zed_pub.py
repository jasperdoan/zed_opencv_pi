import rclpy                        # Python Client Library for ROS 2
from rclpy.node import Node         # Handles the creation of nodes
from sensor_msgs.msg import Image   # Image is the message type
from cv_bridge import CvBridge      # Package to convert between ROS and OpenCV Images
import cv2                          # OpenCV library
 
class ZedPublisher(Node):
  def __init__(self):
    super().__init__('zed_pub')
    self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
    timer_period = 0.1  # seconds
      
    # Create the timer
    self.timer = self.create_timer(timer_period, self.timer_callback)
         
    # Create a VideoCapture object
    # The argument '0' gets the default webcam.
    self.cap = cv2.VideoCapture(0)
         
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
   
  def timer_callback(self):
    # Capture frame-by-frame
    # This method returns True/False as well
    # as the video frame.
    ret, frame = self.cap.read()

    # Cut the frame in half to make it smaller
    frame = frame[0:480, 0:640]

    # Detect aruco markers and draw them on the frame
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    parameters =  cv2.aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
    frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)

    # Convert the OpenCV image to a ROS Image message and publish it
    if ret == True:
      self.publisher_.publish(self.br.cv2_to_imgmsg(frame, "bgr8"))
 
    # Display the message on the console
    self.get_logger().info('🤧 Publishing video frame 😮‍💨')
  


def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  zed_pub = ZedPublisher()
  
  # Spin the node so the callback function is called.
  rclpy.spin(zed_pub)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  zed_pub.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()