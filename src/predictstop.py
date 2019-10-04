#!/usr/bin/env python
from __future__ import print_function

import math
import roslib
import numpy as np
roslib.load_manifest('sbip')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float64,Int32
class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher("ball_prediction",Image,queue_size = 10)
        rospy.Subscriber('ballposefiltered',Pose,self.bpos)
        rospy.Subscriber('balltwistfiltered',Twist,self.btwis)
        rospy.Subscriber('ballaccelfiltered',Twist,self.bacce)
        self.pts = []
        self.x = 0
        self.y = 0
        self.xd = 0
        self.yd = 0
        self.xdd = 0
        self.f = 0
        self.ydd = 0
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/fieldroi",Image,self.callback)

    def bpos(self,msg):
        self.x = msg.position.x
        self.y = msg.position.y

    def btwis(self,msg):
        self.xd = msg.linear.x
        self.yd = msg.linear.y

    def bacce(self,msg):
        self.xdd = msg.linear.x
        self.ydd = msg.linear.y


    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.xdd != 0 and self.ydd != 0 and self.xdd*self.xd <0 and self.ydd*self.yd<0:
            x = self.x - self.xd**2/(2*self.xdd)
            y = self.y - self.yd**2/(2*self.ydd)
            cv2.circle(cv_image, (int(x)+320, 283-int(y)), 50 ,(0,0,255))
            self.pts.append((int(x)+320, 283-int(y)))
            self.f = 1
        else:
            self.f = 0
        for j in self.pts:
            cv2.circle(cv_image, j, 2 ,(0,0,255),-1)
        pts = np.array(self.pts)
        pt = np.mean(pts,axis= 0)
        print(pt)
        cv2.circle(cv_image, (int(pt[0]),int(pt[1])), 2 ,(255,0,0),-1)
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)  
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)