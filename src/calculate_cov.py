#!/usr/bin/env python
import rospy
import numpy as np 
import time
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose 
from std_msgs.msg import Float64,Int32

time = 0
zx = np.array([0.0,0.0])
zy = np.array([0.0,0.0])

def tick(msg):
    global time
    time = msg.data

def bpose(msg):
    global zx
    global zy
    zx[0] = msg.position.x*1.0
    zy[0] = msg.position.y*1.0

def btwist(msg):
    global zx
    global zy
    zx[1] = msg.linear.x*1.0
    zy[1] = msg.linear.y*1.0

class state:
    def __init__(self,P,Q,R):
        global time
        self.R= R
        self.P= P
        self.dt=0.0
        self.time=0.0
        self.to=0.0
        self.I = np.identity(2)
        self.A = np.array([[1,self.dt/1000.0],[0,1]])
        self.B = np.array([[self.dt,0],[0,0]])
        self.X_est = np.array([0,0])
        self.KG = self.P.dot(np.linalg.inv(self.P + self.R))


    def kalman_filter(self,z):
        global time
        #Kalman filter
        self.dt = time-self.to
        self.A = np.array([[1,self.dt/1000.0],[0,1]])
        self.Z = z

        #prediction
        self.X_pred=self.A.dot(self.X_est)
        self.P=self.A.dot(self.P.dot((self.A).T))

        #update
        self.KG=self.P.dot(np.linalg.inv(self.P + self.R))
        self.X_est=self.X_pred + self.KG.dot(self.Z - self.X_pred)
        self.P= (self.I - self.KG)
        self.P= self.P.dot(self.P)
        self.to= time
        return self.X_est

def run():
    global zx
    global zy
    P = np.array([[1,0],[0,0.5]])
    Q = P
    R = P + np.array([[0,0],[0,0.5]])
    x = state(P,Q,R)
    y = state(P,Q,R)
    rospy.Subscriber('bot1pose',Pose,bpose)
    rospy.Subscriber('Time',Float64,tick)
    rospy.Subscriber('bot1twistmeas',Twist,btwist)
    a = rospy.Publisher('/xspeed',Float64,queue_size=10)
    b = rospy.Publisher('/yspeed',Float64,queue_size=10)
    rate = rospy.Rate(20)
    xs = []
    ys = []
    i = 0
    while(i<1000):
        print(zx,zy)
        xs.append(zx*1.0)
        ys.append(zy*1.0)
        i = i + 1
        rate.sleep()
    xs = np.array(xs,dtype=float)
    ys = np.array(ys,dtype=float)
    xs = xs.T
    ys = ys.T
    print('xs',xs)
    print('ys',ys)
    sigx = np.cov(xs)
    sigy = np.cov(ys)
    print('x',sigx)
    print('y',sigy)

if __name__ == '__main__':
    rospy.init_node('KF', anonymous=True)
    run()