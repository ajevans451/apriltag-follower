
import rclpy
from typing import List
import cv2
import numpy as np
from dt_apriltags import Detector
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
from cv_bridge import CvBridge

SHOWVISUALS=True

class TagFollower(Node):

    def __init__(self, image_topic):
        super().__init__('tag_follower')

        self.cv_image = None                        # the latest image from the camera
        self.bridge=CvBridge()                  # used to convert ROS messages to OpenCV
        self.detector = Detector("tag36h11")
        self.create_subscription(Image, image_topic, self.process_image, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        image = self.cv_image
        #image = cv2.imread(self.cv_image, cv2.IMREAD_GRAYSCALE)
        detections = self.detector.detect(image)

        if SHOWVISUALS==True:
            cv2.imshow("Greyscale Original",image) # greyscale image
            k=cv2.waitKey(0)
            if k==27:
                cv2.destroyAllWindows

        color_image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
        for detection in detections:
            for idx in range(len(detection.corners)):
                cv2.line(color_image, tuple(detection.corners[idx-1, :].astype(int)), tuple(detection.corners[idx, :].astype(int)), (0,255,0), 3)

        if SHOWVISUALS==True:
            cv2.imshow("Tags Detected", color_image)
            k=cv2.waitKey(0)
            if k==27:
                cv2.destroyAllWindows()


def main(args=None):
    rclpy.init()
    n = TagFollower("camera/image_raw")
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

if __name__ == '__main__':
    node = TagFollower("/camera/image_raw")
    node.run()
