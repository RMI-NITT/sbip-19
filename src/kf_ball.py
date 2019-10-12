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

time = 0
zx = np.array([0,0,0])
zy = np.array([0,0,0])

def bpos(msg):
    global time
    global zx
    global zy
    zx[0] = msg.position.x
    zy[0] = msg.position.y

def btwis(msg):
    global time
    global zx
    global zy
    zx[1] = msg.linear.x
    zy[1] = msg.linear.y

def bacce(msg):
    global time
    global zx
    global zy
    zx[2] = msg.linear.x
    zy[2] = msg.linear.y

def tick(msg):
    global time
    time = msg.data

class state:
    def __init__(self,P,Q,R):
        global time
        self.R= R
        self.P= P
        self.dt=0.0
        self.time=0.0
        self.to=0.0
        self.I = np.identity(3)
        self.A = np.array([[1,self.dt/1000.0,self.dt**2/2000000],[0,1,self.dt/1000.0],[0,0,1]])
        self.X_est = np.array([0,0,0])
        self.KG = self.P.dot(np.linalg.inv(self.P + self.R))
        self.a = 0


    def kalman_filter(self,z):
        global time
        #Kalman filter
        self.dt = time-self.to
        self.A = np.array([[1,self.dt/1000.0,0],[0,1,0],[0,0,1]])
        self.Z = z

        #prediction
        xold = self.X_est
        self.X_pred=self.A.dot(self.X_est)
        self.P=self.A.dot(self.P.dot((self.A).T))

        #update
        self.KG=self.P.dot(np.linalg.inv(self.P + self.R))
        self.X_est=self.X_pred + self.KG.dot(self.Z - self.X_pred)
        self.P= (self.I - self.KG)
        self.P= self.P.dot(self.P)
        self.a = (self.X_est[1] - xold[1])*1000.0/self.dt
        self.to= time
        return self.X_est

def run():
    global zx
    global zy
    P = np.array([[1,0,0],[0,1,0],[0,0,1]])
    Q = np.array([[1,0,0],[0,1,0],[0,0,1]])
    R = np.array([[1,0,0],[0,1,0],[0,0,1]])
    x = state(P,Q,R)
    y = state(P,Q,R)
    rospy.Subscriber('ballpose',Pose,bpos)
    rospy.Subscriber('balltwist',Twist,btwis)
    rospy.Subscriber('ballaccela',Twist,bacce)
    bpose = rospy.Publisher('ballposefiltered',Pose,queue_size=10)
    btwist = rospy.Publisher('balltwistfiltered',Twist,queue_size=10)
    baccel = rospy.Publisher('ballaccelfiltered',Twist,queue_size=10)
    rospy.Subscriber('Time',Float64,tick)
    rate = rospy.Rate(20)
    bp = Pose()
    bt = Twist()
    ba = Twist()
    while(True):
        X = x.kalman_filter(zx)
        Y = y.kalman_filter(zy)
        bp.position.x = X[0]
        bp.position.y = Y[0]
        bt.linear.x = X[1]
        bt.linear.y = Y[1]
        ba.linear.x = x.a
        ba.linear.y = y.a
        print(np.sqrt(x.a**2+ y.a**2))
        bpose.publish(bp)
        btwist.publish(bt)
        baccel.publish(ba)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('KF', anonymous=True)
    run()