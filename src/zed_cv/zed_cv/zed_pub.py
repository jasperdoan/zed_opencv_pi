import rclpy                                    # Python Client Library for ROS 2
from rclpy.node import Node                     # Handles the creation of nodes
from rclpy.qos import qos_profile_sensor_data   # Quality of Service profile for sensor data
from sensor_msgs.msg import Image               # Image is the message type
import cv2                                      # OpenCV library
from cv_bridge import CvBridge                  # Package to convert between ROS and OpenCV Images
import numpy as np                              # Create the node and spin it



class ZedPublisher(Node):
    def __init__(self):
        super().__init__('zed_pub')
        self.publisher = self.create_publisher(Image, 'zed/images', qos_profile=qos_profile_sensor_data)
        timer_period = 1/30                                                             # FPS = 1 / timer_period

        self.timer = self.create_timer(timer_period, self.timer_callback)               # Create a timer with the callback function
        self.cap = cv2.VideoCapture(0)                                                  # Initialize the camera
        self.br = CvBridge()                                                            # Used to convert between ROS and OpenCV Images



    def timer_callback(self):
        if self.count_subscribers('zed/images') > 0:                                    # Check if there are subscribers
            success, frame = self.cap.read()                                            # Read a frame from the camera

            if success:                                                                 # If there is a frame
                # # ==== Aruco detection =================================================
                # # ======================================================================
                # frame = np.split(frame, 2, axis=1)[0]                                   #     Split the frame in two
                # frame = cv2.resize(frame, (640, 360), interpolation=cv2.INTER_AREA)     #     Resize the frame

                # aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)            #     Create the aruco dictionary
                # parameters = cv2.aruco.DetectorParameters_create()                      #     Create the aruco parameters
                # corners, ids = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)           #     Detect the markers
                # frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)              #     Draw the markers
                # # ======================================================================

                self.get_logger().info('🤧 Publishing Zed frames 😮‍💨')               
                self.publisher.publish(self.br.cv2_to_compressed_imgmsg(frame))         # Publish the frame

            else:                                                                       # If there is no frame             
                self.get_logger().info(f'😭 Unsuccessful frames capture 😭')  
        
        else:                                                                           # If there are no subscribers                  
            self.get_logger().info(f'🥺 No subscribers to receive message 🥺')




def main(args=None):

    rclpy.init(args=args)                                          # Initialize the ROS client library for Python
    zed_pub = ZedPublisher()                                       # Create the node

    try:                                                           # Try to spin the node
        rclpy.spin(zed_pub)                                        #     Spin the node
    except Exception as e:                                         # Catch any error that occurs
        zed_pub.get_logger().info(f'😭 Error: {e} 😭')             #     Print the error
    finally:                                                       # Finally
        zed_pub.cap.release()                                      #     Release the camera
        zed_pub.destroy_node()                                     #     Destroy the node
        rclpy.shutdown()                                           #     Shutdown the ROS client library for Python

if __name__ == '__main__':
    main()