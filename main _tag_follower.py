
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
    """
    The TagFollower class will take images from the Neato's camera,
    detect any AprilTags within it, and then send linear and angular 
    motion commands to the Neato to head towards the furthest one
    """
    def __init__(self, image_topic):
        super().__init__('tag_follower')

        self.cv_image = None                        # the latest image from the camera
        self.bridge=CvBridge()                  # used to convert ROS messages to OpenCV
        self.detector = Detector("tag36h11")    #AprilTag detector set specifically to the kind of AprilTag we are looking for
        self.create_subscription(Image, image_topic, self.run_loop, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def run_loop(self, msg):
        """
        The main loop of the robot controller, should be constantly running while
        attempting to follow tags with the neato
        """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        image = self.cv_image
        detections = self.detector.detect(image,True,[585.0182, 584.6872, 377.3953, 225.2862],.173)

        #The following section of code allows for visual checks on each run of the loop
        #to visualize each detected AprilTag
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


        (angle,distance)=self.obtain_heading(image) #Finds the direction the Neato should go to get to the furthest AprilTag
        self.go_to_tag(angle,distance) #Commands the Neato to go to that tag


    def obtain_heading(self, img):     
        """
        Takes in an image of what the Neato currently sees, determines which
        detected AprilTag is the furthest away, and returns the distance to
        that tag as well as the angle between the tag and the center of the
        Neato's vision
        """ 

        #Detects any tags in the current image, camera intrinsics inputted into this line
        tags = self.detector.detect(self.cv_image,True,[585.0182, 584.6872, 377.3953, 225.2862],0.173) 
        if not tags:    #Tell the neato to drive forward if no tag is seen
            return 0,1 
        tag_z=[]
        tag_x=[]
        tag_ids=[]        
        for tag in tags:    #Find the z (forward) and x (perpendicular) distances to each tag along with the tag id
            tag_z.append(tag.pose_t[2])
            tag_ids.append(tag.tag_id)
            tag_x.append(tag.pose_t[0])

        
        target_z=tag_z[tag_z.index(max(tag_z))]    
        #target_z is straight forward distance to furthest tag
        target_x=tag_x[tag_z.index(max(tag_z))]+1   
        #target_x is horizontal distance to furthest tag (+1 to keep robot to the right of the tag so as not to hit a pillar)
        angle=np.tanh(target_x/target_z)            #angle determined using trig identities
        full_distance=np.sqrt(target_z**2+target_x**2)  #full distance to tag found using Pythagorean theorem
        return angle, full_distance
        

    def go_to_tag(self,angle, distance):
        """
        Takes in angle and distance to tag and commands Neato motors to head towards that position
        """
        drive_msg = Twist()
        drive_msg.linear.x = np.clip(float(FORWARDCONSTANT*distance), -1.0, 1.0)    #Determines final linear drive message
        drive_msg.angular.z = np.clip(float(TURNCONSTANT*angle), -1.0, 1.0)         #Determines final angular drive message
        self.pub.publish(drive_msg) #Commands Neato motors
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
