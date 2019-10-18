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
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher("ball_prediction",Image,queue_size = 10)
        rospy.Subscriber('/kf_ballpose',Pose, self.bpos)
        rospy.Subscriber('/kf_ball_velocity',Twist, self.btwis)
        rospy.Subscriber('/kf_ball_acceleration',Twist, self.bacce)
        rospy.Subscriber('/ball_Px', numpy_msg(Floats), self.Px_cb)
        rospy.Subscriber('/ball_Py', numpy_msg(Floats), self.Py_cb)
        rospy.Subscriber("/Time", Float64, self.time_cb)
        self.pts = []
        self.X = np.matrix([[0],[0], [0]], dtype = float)
        self.Y = np.matrix([[0],[0], [0]], dtype = float)
        self.Px = np.eye(3, dtype = float)
        self.Py = np.eye(3, dtype = float)
        self.Q = np.eye(3, dtype = float)
        self.Q[0,0] = 1
        self.Q[1,1] = 120
        self.Q[2,2] = 1440
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/fieldroi",Image,self.callback)
        self.del_t = 0.0
        self.t = 0.0

    def time_cb(self, info):
        self.del_t = (float(info.data) - self.t) / 100
        self.t = float(info.data)
    
    def bpos(self,msg):
        self.X[0] = msg.position.x
        self.Y[0] = msg.position.y

    def btwis(self,msg):
        self.X[1] = msg.linear.x
        self.Y[1] = msg.linear.y

    def bacce(self,msg):
        self.X[2] = msg.linear.x
        self.Y[2] = msg.linear.y

    def Px_cb(self,msg):
        self.Px = np.reshape(msg.data, (3,3))
        
    def Py_cb(self,msg):
        self.Py = np.reshape(msg.data, (3,3))

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.A = np.matrix([[1, self.del_t, ((self.del_t**2)/2)],[0, 1, self.del_t], [0, 0, 1]], dtype=float)
        self.future_X = self.X
        self.future_Y = self.Y
        self.future_Px = self.Px[0,0]
        self.future_Py = self.Py[0,0]
        self.stop_t = 0
        self.stop_frame_count = 0

        if(math.sqrt(self.X.item(1)**2 + self.Y.item(1)**2) > 3):
          if(( (self.X.item(1)*self.X.item(2))+ (self.Y.item(1)*self.Y.item(2)) <0) ):
            self.stop_t = math.sqrt(self.X.item(1)**2 + self.Y.item(1)**2)   / math.sqrt(self.X.item(2)**2 + self.Y.item(2)**2)
            self.stop_frame_count = int(self.stop_t/self.del_t) + 1
            self.A = np.matrix([[1, (self.stop_frame_count*self.del_t), (((self.stop_frame_count*self.del_t)**2)/2)],[0, 1, (self.stop_frame_count*self.del_t)], [0, 0, 1]], dtype=float)
            self.future_X = np.matmul(self.A, self.X)
            self.future_Px = np.matmul(np.matmul(np.transpose(self.A), self.Px),self.A).item(0) + self.stop_frame_count*self.Q[0,0]
            self.future_Y = np.matmul(self.A, self.Y)
            self.future_Py = np.matmul(np.matmul(np.transpose(self.A), self.Py),self.A).item(0) + self.stop_frame_count*self.Q[0,0]
          self.x_stop = self.future_X[0]
          self.y_stop = self.future_Y[0]
        else:
          self.x_stop = self.X[0]
          self.y_stop = self.Y[0]
        
        self.radius = int(math.sqrt(self.future_Px.item(0)**2 + self.future_Py.item(0)**2))
        cv2.circle(cv_image, (int(self.x_stop)+320, 283-int(self.y_stop)), self.radius, (0, 0, 255))
        cv2.circle(cv_image, (int(self.x_stop)+320, 283-int(self.y_stop)), 8, (0, 0, 255), thickness=-1)
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