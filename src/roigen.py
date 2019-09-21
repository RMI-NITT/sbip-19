#!/usr/bin/env python
from __future__ import print_function

import roslib
import numpy as np
roslib.load_manifest('sbip')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#p1,p2,p3,p4

points_1 = np.float32([(95,16),(588,8),(98,477),(636,454)]) 
points_2 = np.float32([(0,0),(640,0),(0,566),(640,566)])

lower_red = np.array([0,220,180])
upper_red = np.array([25,255,200])

class image_converter:
  def getcorner(self):
    mask_red = cv2.inRange(self.cv_image,np.array([0,0,100]),np.array([50,50,250]))
    kernel =np.ones((10,10),np.uint8)
    points = []
    dilation = cv2.dilate(mask_red,kernel,iterations = 1)
    _,cnts,_ = cv2.findContours(dilation, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in cnts:
      M = cv2.moments(c)
      cX = int(M["m10"] / M["m00"])
      cY = int(M["m01"] / M["m00"])
      points.append((cX,cY))

#This section orders the points clockwise(starting from bottom left) for the perspective transform by using the sums and differences of x and y coordinates
    self.points = np.float32(points)
    self.pts = np.float32(points)
    d = np.diff(self.points, axis=1)
    s = self.points.sum(axis=1)

    self.pts[0] = self.points[np.argmin(s)]
    self.pts[2] = self.points[np.argmax(d)]
    self.pts[1] = self.points[np.argmin(d)]
    self.pts[3] = self.points[np.argmax(s)]

 
  def __init__(self):
    self.image_pub = rospy.Publisher("fieldroi",Image, queue_size = 10)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("usb_cam/image_raw",Image,self.callback)
    self.i = 0

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      self.cv_image = cv_image
    except CvBridgeError as e:
      print(e)
    frame = cv2.medianBlur(cv_image,3)
    if self.i == 0:
      #self.getcorner()
      self.i = 1
    perspective_transform = cv2.getPerspectiveTransform(points_1,points_2)
    dst = cv2.warpPerspective(frame,perspective_transform,(640,566))

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(dst, "bgr8"))
    except CvBridgeError as e:
      print(e)


def main(args):
  rospy.init_node('roigen', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
