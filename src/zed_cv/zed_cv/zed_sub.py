import os                                                   # Import the OS module for file path manipulation
import sys                                                  # Import the sys module for command line arguments
from datetime import datetime                               # Import the datetime module for timestamping
import rclpy                                                # Python library for ROS 2
from rclpy.node import Node                                 # Handles the creation of nodes
from rclpy.callback_groups import ReentrantCallbackGroup    # Handles the creation of callback groups
from rclpy.qos import qos_profile_sensor_data               # Quality of Service profile for sensor data
from sensor_msgs.msg import Image                           # Image is the message type
import cv2                                                  # OpenCV library
from cv_bridge import CvBridge                              # Package to convert between ROS and OpenCV Images
import numpy as np                                          # Python library for scientific computing



class ZedSubscriber(Node):
    def __init__(self):
        super().__init__('zed_sub')                                                                             # Create the node

        self.callback_group = ReentrantCallbackGroup()                                                          # Create a callback group

        # Create a subscriber with the appropriate topic, message type, and callback function
        self.subscription = self.create_subscription( Image, 'zed/images', self.listener_callback, 
                                                      qos_profile=qos_profile_sensor_data,
                                                      callback_group=self.callback_group)
        self.subscription                                                                                       # Prevent unused variable warning   
        
        self.br = CvBridge()                                                                                    # Used to convert between ROS and OpenCV images
        frame_width = 640                                                                                       # Set the frame width
        frame_height = 360                                                                                      # Set the frame height

        self.frame = np.zeros([frame_height, frame_width, 4], dtype=np.uint8)                                   # Create a blank image
        self.media_path = os.path.join(os.path.dirname(os.path.realpath(sys.argv[0])), 'recordings', 'zed2i')   # Set the path to save the video

        try:                                                                                                    # Try to create the directory   
            os.makedirs(self.media_path, exist_ok=True)                                                         # Create the directory if it does not exist   
        except OSError as e                                                                                     # Catch any error that occurs
            self.get_logger().info(f'{e}')                                                                      # Print the error

        now = datetime.now().strftime('%m-%d-%Y_%H:%M:%S')                                                      # Get the current date and time
        self.out = cv2.VideoWriter(os.path.join(self.media_path, 'ZED2i_'+now+'.mp4'),                          # Create the video writer
                                   cv2.VideoWriter_fourcc(*'mp4v'), 30, (frame_width, frame_height))            # Set the video codec, frame rate, and frame size

        self.get_logger().info('ZED Subscriber has been initialized')                                           # Print status message
        self.snapshot = False                                                                                   # Set the snapshot flag to false
        self.gui = False                                                                                        # Set the GUI flag to false
        self.verbose = False                                                                                    # Set the verbose flag to false
     


    def listener_callback(self, data):
        self.frame = self.br.compressed_imgmsg_to_cv2(data)                                                     # Convert the ROS image message to an OpenCV image
        self.out.write(self.frame)                                                                              # Write the frame to the video file

        if self.verbose:                                                                                        # If the verbose flag is set
            self.get_logger().info('😩 Receiving video frame!! 😫')                                            # Display the message on the console
        
        if self.snapshot:                                                                                       # If the snapshot flag is set
            now = datetime.now().strftime('%m-%d-%Y_%H:%M:%S')                                                  # Get the current date and time
            cv2.imwrite(os.path.join(self.media_path, 'ZED2i_'+now+'.png'), self.frame)                         # Save the image
            self.snapshot = False                                                                               # Reset the snapshot flag

        if self.gui:                                                                                            # If the GUI flag is set
            cv2.imshow('ZED2i View', self.frame)                                                                # Display the image in a window
            cv2.waitKey(1)                                                                                      # Wait for a key press





def main(args=None):
    rclpy.init(args=args)                                           # Initialize the rclpy library

    zed_subs = ZedSubscriber()                                      # Create the node
    zed_subs.gui = True                                             # Set the GUI flag to true
    cv2.namedWindow('ZED2i View', cv2.WINDOW_NORMAL)                # Create a window

    try:
        rclpy.spin(zed_subs)                                        # Spin the node
    except Exception as e:                                          # Catch any error that occurs
        zed_subs.get_logger().info(f'😭 {e} 😭')                   #     Print the error
    except KeyboardInterrupt:                                       # Catch a keyboard interrupt
        zed_subs.get_logger().info(f'😭 Keyboard Interrupt 😭\n')  #     Print the message
    finally:                                                        # Always do the following
        if zed_subs.gui:                                            # If the GUI flag is set
            cv2.destroyAllWindows()                                 #     Destroy all windows
        zed_subs.out.release()                                      # Release the video writer
        zed_subs.destroy_node()                                     # Destroy the node
        rclpy.shutdown()                                            # Shutdown the rclpy library



if __name__ == '__main__':
    main()