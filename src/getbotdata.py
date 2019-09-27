#!/usr/bin/env python
from __future__ import print_function

import roslib
import imutils
import numpy as np
roslib.load_manifest('sbip')
import sys
import rospy
import cv2
import math as m
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge, CvBridgeError

lower_white = np.array([(180,180,170)])
higher_white = np.array([(255,255,255)])
lower_yellow = np.array([(120,230,220)])
higher_yellow = np.array([(155,255,255)])

rows = 640
cols = 480

class botroi:
  def __init__(self,img,x,y):
    self.img = img.copy()
    self.x = x
    self.y = y
    self.x,self.y = ftransform(self.x,self.y)
    _, contours, hierarchy = cv2.findContours(self.img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    self.botn = hierarchy.shape[1] -2
    maxarea = 0
    maxarea2 = 0
    index = 0
    for i in range(self.botn+2):
      area = cv2.contourArea(contours[i])
      if area > maxarea:
        maxarea2 = maxarea
        maxarea = area
        index2 = index
        index = i
      if area > maxarea2 and area<maxarea:
        maxarea2 = area
        index2 = i
    M = cv2.moments(contours[index2])
    if M["m00"] > 0 or M["m00"] < 0:
      cX = int(M["m10"] / M["m00"])
      cY = int(M["m01"] / M["m00"])
      dx = cX 
      dy = cY 
      dx,dy = smfttramsform(dx,dy)
      self.dx = dx + self.x
      self.dy = dy + self.y
      dist = m.sqrt((self.dx - self.x)**2 + (self.dy - self.y)**2)
      dify = self.dy - self.y
      sin = dify/dist
      difx = self.dx - self.x
      cos = difx/dist
      self.theta = m.atan2(sin,cos)

  def returnpose(self):
    pose = Pose()
    pose.position.x = self.x
    pose.position.y = self.y
    yaw = m.tan(self.theta/2)
    pose.orientation.z = yaw
    pose.orientation.w = 1
    return pose

      

def smfttramsform(x,y):
  x = (x -35)
  y = (35 -y)
  return x,y

def ftransform(x,y):
  x = (x-640/2)
  y = (566/2-y)
  return x,y

def printdata(bots):
  print('bot',bots[0].botn,'centre',bots[0].x,bots[0].y,'dribbler',bots[0].dx,bots[0].dy,'theta',bots[0].theta)
  print('bot',bots[1].botn,'centre',bots[1].x,bots[1].y,'dribbler',bots[1].dx,bots[1].dy,'theta',bots[1].theta)
  print('bot',bots[2].botn,'centre',bots[2].x,bots[2].y,'dribbler',bots[2].dx,bots[2].dy,'theta',bots[2].theta)
  print('bot',bots[3].botn,'centre',bots[3].x,bots[3].y,'dribbler',bots[3].dx,bots[3].dy,'theta',bots[3].theta)

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("botmask",Image, queue_size = 10)
    self.bot1pub = rospy.Publisher("bot1pose",Pose, queue_size = 10)
    self.bot2pub = rospy.Publisher("bot2pose",Pose, queue_size = 10)
    self.bot3pub = rospy.Publisher("bot3pose",Pose, queue_size = 10)
    self.bot4pub = rospy.Publisher("bot4pose",Pose, queue_size = 10)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/fieldroi",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    cv_image  = cv2.GaussianBlur(cv_image,(3,3),0)
    mask_white = cv2.inRange(cv_image,lower_white,higher_white)
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    th2 = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,11,2)
    th6 = cv2.bitwise_and(th2,mask_white)
    _,cnts,_ = cv2.findContours(th6, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in cnts:
      M = cv2.moments(c)
      cX = int(M["m10"] / M["m00"])
      cY = int(M["m01"] / M["m00"])
      cv2.circle(th6, (cX, cY), 30, (255, 255, 255), -1)
    th7 = cv2.bitwise_and(th6,mask_white)
    cnts = cv2.findContours(th7.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    self.bots = []
    for c in cnts:
      M = cv2.moments(c)
      if M["m00"] > 0 or M["m00"] < 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        self.bots.append(botroi(mask_white[cY-35:cY+35,cX-35:cX+35],cX,cY))
    #printdata(self.bots)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(th7, "mono8"))
      self.publishdata()
    except CvBridgeError as e:
      print(e)

  def publishdata(self):
    for i in range(4):
      n = self.bots[i].botn
      if n ==1:
        self.bot1pub.publish(self.bots[i].returnpose())
      if n ==2:
        self.bot2pub.publish(self.bots[i].returnpose())
      if n ==3:
        self.bot3pub.publish(self.bots[i].returnpose())
      if n ==4:
        self.bot4pub.publish(self.bots[i].returnpose())

def main(args):
  rospy.init_node('botdatapub', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
