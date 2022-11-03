# Basic ROS 2 program to subscribe to real-time streaming
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

# Import the necessary libraries
import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image, CompressedImage  # Image is the message type
from cv_bridge import CvBridge,CvBridgeError  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
from datetime import datetime
import base64 
import os

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
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(CompressedImage, 'video_frames/compressed_image', 10)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

    def listener_callback(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        self.get_logger().info('Receiving video frame')

        self.republish_compressed(data)

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data)

        # Display image
        cv2.imshow("camera", current_frame)

        cv2.waitKey(1)

    def republish_compressed(self, data):
        # try:
        cv_image = self.br.imgmsg_to_cv2(data,"8UC3")
        now = datetime.now() # current date and time
        date_time = now.strftime("%m-%d-%Y %H:%M:%S")
        fileName = "foo_"+date_time
        fullPath =""+fileName+".jpg"
        cv2.imwrite(fullPath, cv_image)
        # cv_image.SaveImage("images/"+fileName+".jpg", cv_image)
        
        image_64 = ""
        with open(fullPath, "rb") as img_file:
          image_64 = base64.b64encode(img_file.read())

        # image_64 = base64.encodestring(
        #     open("images/"+fileName+".jpg", "rb").read())
        self.publisher_.publish(image_64)

        os.remove(fullPath)
        
        print('compreseed success')

        # except CvBridgeError:
        #     print('error compreseed')


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
