
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

SHOWVISUALS=False
FORWARDCONSTANT=.5
TURNCONSTANT=-5

class TagFollower(Node):

    def __init__(self, image_topic):
        super().__init__('tag_follower')

        self.cv_image = None                        # the latest image from the camera
        self.bridge=CvBridge()                  # used to convert ROS messages to OpenCV
        self.detector = Detector("tag36h11")
        self.create_subscription(Image, image_topic, self.run_loop, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def run_loop(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        image = self.cv_image
        #image = cv2.imread(self.cv_image, cv2.IMREAD_GRAYSCALE)
        detections = self.detector.detect(image,True,[585.0182, 584.6872, 377.3953, 225.2862],.173)

        if SHOWVISUALS==True:
            cv2.imshow("Greyscale Original",image) # greyscale image
            k=cv2.waitKey(0)
            if k==27:
                cv2.destroyAllWindows

        color_image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
        for detection in detections:
            for idx in range(len(detection.corners)):
                cv2.line(color_image, tuple(detection.corners[idx-1, :].astype(int)), tuple(detection.corners[idx, :].astype(int)), (0,255,0), 3)
            #print(detection.pose_t[2])

        if SHOWVISUALS==True:
            cv2.imshow("Tags Detected", color_image)
            k=cv2.waitKey(0)
            if k==27:
                cv2.destroyAllWindows()

 
        (angle,distance)=self.obtain_heading(image)
        self.go_to_tag(angle,distance)


    def obtain_heading(self, img):      
        tags = self.detector.detect(self.cv_image,True,[585.0182, 584.6872, 377.3953, 225.2862],0.173)
        if not tags:
            return 0,-.5
        tag_z=[]
        tag_x=[]
        tag_ids=[]        
        for tag in tags:
            tag_z.append(tag.pose_t[2])
            tag_ids.append(tag.tag_id)
            tag_x.append(tag.pose_t[0])

        
        target_z=tag_z[tag_z.index(max(tag_z))]
        target_x=tag_x[tag_z.index(max(tag_z))]+0.25
        angle=np.tanh(target_x/target_z)
        full_distance=np.sqrt(target_z**2+target_x**2)
#        print("furthest_tag: ", furthest_tag)
#        print("target z:", target_z)
#        print("target x:", target_x)
        return angle, full_distance
        

    def go_to_tag(self,angle, distance):
        print(angle)
        drive_msg = Twist()
        drive_msg.linear.x = np.clip(float(FORWARDCONSTANT*distance), -1.0, 1.0)
        drive_msg.angular.z = np.clip(float(TURNCONSTANT*angle), -1.0, 1.0)
        self.pub.publish(drive_msg)
        return

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
