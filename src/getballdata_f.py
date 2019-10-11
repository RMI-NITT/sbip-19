#!/usr/bin/env python
############
#INCOMPLETE#
############
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

lower_white = np.array([(230,230,230)])
higher_white = np.array([(255,255,255)])
lower_yellow = np.array([(0,230,220)])
higher_yellow = np.array([(10,255,255)])

def ftransform(x,y):
  x = (x-640/2)
  y = (566/2-y)
  return x,y

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("ballmask",Image, queue_size = 10)
    self.kf_ballpub = rospy.Publisher("kf_ballpose",Pose, queue_size = 10)
    self.kf_ballpub2 = rospy.Publisher("kf_ball_velocity", Twist, queue_size = 10)
    self.kf_ballpub3 = rospy.Publisher("kf_ball_acceleration", Twist, queue_size = 10)
    self.kf_ballpub4 = rospy.Publisher("kf_ball_stop", Pose, queue_size = 10)
    self.bridge = CvBridge()
    self.image_pub = rospy.Publisher("ball_prediction",Image, queue_size = 10)
    self.image_sub = rospy.Subscriber("/fieldroi",Image,self.callback)
    self.timesub = rospy.Subscriber("/usb_cam/camera_info", CameraInfo, self.camcb)
    self.t_flag = 0
    self.init_flag = 0
    self.X = np.matrix([[0],[0], [0]], dtype = float)
    self.Y = np.matrix([[0],[0], [0]], dtype = float)
    self.Px = np.eye(3, dtype = float)
    self.Py = np.eye(3, dtype = float)
    self.Q = np.eye(3, dtype = float)
    self.Q[0,0] = 1
    self.Q[1,1] = 15
    self.Q[2,2] = 225
    self.H = np.matrix([1, 0, 0], dtype = float)
    self.R = 4*np.eye(1, dtype = float)
    self.Zx = np.eye(1, dtype = float)
    self.Zy = np.eye(1, dtype = float)
    self.del_t = 0.067
  
  def camcb(self, msg):
    if self.t_flag == 0:
        self.t0 = msg.header.seq
        self.t_s = self.t0
        self.t_flag = 1
    self.t = msg.header.seq - self.t0

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    
    mask_white = cv2.inRange(cv_image,lower_yellow,higher_yellow)
    _,cnts,_ = cv2.findContours(mask_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in cnts:
      M = cv2.moments(c)
      cX = int(M["m10"] / M["m00"])
      cY = int(M["m01"] / M["m00"])
      cX,cY = ftransform(cX,cY)
      bp = Pose()
      bp.position.x = cX
      bp.position.y = cY
      
    fbp = Pose()  
    fbv = Twist()
    fba = Twist()
    fbs = Pose()
    
    if self.init_flag == 0:

        self.Zx[0] = bp.position.x
        self.Zy[0] = bp.position.y
        fbp.position.x = bp.position.x
        fbp.position.y = bp.position.y
        self.init_flag = 1
    else:
        self.A = np.matrix([[1, self.del_t, ((self.del_t**2)/2)],[0, 1, self.del_t], [0, 0, 1]], dtype=float)
        
        self.X_prev = self.X
        self.X = np.matmul(self.A,self.X_prev)
        self.Px = np.matmul(np.matmul(self.A,self.Px),np.transpose(self.A)) + self.Q
        self.Zx[0] = bp.position.x

        self.gain_x = np.matmul(np.matmul(self.Px, np.transpose(self.H)), np.linalg.inv(np.matmul(np.matmul(self.H,self.Px),np.transpose(self.H)) + self.R))
        self.X = self.X + np.matmul(self.gain_x, (self.Zx - np.matmul(self.H, self.X)))
        self.Px = np.matmul((np.eye(3,dtype=float)- np.matmul(self.gain_x,self.H)),self.Px)
        
        self.Y_prev = self.Y
        self.Y = np.matmul(self.A,self.Y_prev)
        self.Py = np.matmul(np.matmul(self.A,self.Py),np.transpose(self.A)) + self.Q
        self.Zy[0] = bp.position.y        
        
        self.gain_y = np.matmul(np.matmul(self.Py, np.transpose(self.H)), np.linalg.inv(np.matmul(np.matmul(self.H,self.Py),np.transpose(self.H)) + self.R))
        self.Y = self.Y + np.matmul(self.gain_y, (self.Zy - np.matmul(self.H, self.Y)))
        self.Py = np.matmul((np.eye(3,dtype=float)-np.matmul(self.gain_y,self.H)),self.Py)
        
        fbp.position.x = float(self.X.item(0))
        fbp.position.y = float(self.Y.item(0))
        fbv.linear.x = float(self.X.item(1))
        fbv.linear.y = float(self.Y.item(1))
        fba.linear.x = float(self.X.item(2))
        fba.linear.y = float(self.Y.item(2))
        
        self.future_X = self.X
        self.future_Y = self.Y
        self.future_Px = self.Px
        self.future_Py = self.Py

        if(math.sqrt(self.X.item(1)**2 + self.Y.item(1)**2) > 3):
          if(( (self.X.item(1)*self.X.item(2))+ (self.Y.item(1)*self.Y.item(2)) <0) ):
            print('decelerating')
            self.count = 0
            while((math.sqrt(self.future_X.item(1)**2 + self.future_Y.item(1)**2) > 20)):
              self.count += 1
              if(self.count > 200):
                break
              self.future_X = np.matmul(self.A, self.future_X)
              self.future_Px = np.matmul(np.matmul(np.transpose(self.A), self.future_Px),self.A) + self.Q
              self.future_Y = np.matmul(self.A, self.future_Y)
              self.future_Py = np.matmul(np.matmul(np.transpose(self.A), self.future_Py),self.A) + self.Q
              print('propagating, count = ', self.count)
          fbs.position.x = self.future_X[0]
          fbs.position.y = self.future_Y[0]
        else:
          fbs.position.x = self.X[0]
          fbs.position.y = self.Y[0]
        
    self.radius = int(math.sqrt(self.future_Px.item(0)**2 + self.future_Py.item(0)**2))
    cv2.ellipse(cv_image, (int(fbs.position.x)+320, 283-int(fbs.position.y)), (int(self.future_Px.item(0)),int(self.future_Py.item(0))),0,0,360,(0,0,255))
    cv2.circle(cv_image, (int(fbs.position.x)+320, 283-int(fbs.position.y)), 1, (0,0,255), thickness=-1)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      self.kf_ballpub.publish(fbp)
      self.kf_ballpub2.publish(fbv)
      self.kf_ballpub3.publish(fba)
      self.kf_ballpub4.publish(fbs)
    except CvBridgeError as e:
      print(e)

def main(args):
  rospy.init_node('balldatapub', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)