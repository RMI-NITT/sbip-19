#!/usr/bin/env python
import rospy
import numpy as np 
import matplotlib.pyplot as plt
import time
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose 
from std_msgs.msg import Float64,Int32

time = 0
zx = np.array([0.0,0.0])
zy = np.array([0.0,0.0])

zxcmd = 0.0
zycmd = 0.0

filteredx = 0.0
filteredy = 0.0

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

def btwistg(msg):
    global zxcmd
    global zycmd
    zxcmd = msg.linear.x*1.0
    zycmd = msg.linear.y*1.0

def da(msg):
    global filteredx
    filteredx = msg.data

def db(msg):
    global filteredy
    filteredy = msg.data


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
    global zxcmd
    global zycmd
    global filteredx
    global filteredy
    P = np.array([[1,0],[0,0.5]])
    Q = P
    R = P + np.array([[0,0],[0,0.5]])
    x = state(P,Q,R)
    y = state(P,Q,R)
    rospy.Subscriber('bot1pose',Pose,bpose)
    rospy.Subscriber('Time',Float64,tick)
    rospy.Subscriber('bot1twistmeas',Twist,btwist)
    rospy.Subscriber('bot1twistglobal',Twist,btwistg)
    rospy.Subscriber('xspeed',Float64,da)
    rospy.Subscriber('yspeed',Float64,db)
    rate = rospy.Rate(20)
    ratio = []
    cmdx = []
    cmdy = []
    measx = []
    measy = []
    filx = []
    fily = []
    za = []
    i = 0
    while(i<1000):
        zcmd = np.sqrt(zxcmd**2 + zycmd**2)
        print zcmd
        if zcmd != 0.0:
            zm = np.sqrt(zx[1]**2 + zy[1]**2)
            z = zm/zcmd
            za.append(0.0)
            cmdx.append(zxcmd*3*10**3)
            measx.append(zx[1])
            filx.append(filteredx)
            cmdy.append(zycmd*3*10**3)
            measy.append(zy[1])
            fily.append(filteredy)
            i = i+1
        rate.sleep()
    print 'done'

    plt.subplot(2, 1, 1)
    plt.plot(cmdx,label='command')
    plt.plot(measx,label='measured')
    plt.plot(filx,label='filtered')
    plt.plot(za)
    plt.legend(loc = 'lower left')
    plt.subplot(2, 1, 2)
    plt.plot(cmdy,label='command')
    plt.plot(measy,label='measured')
    plt.plot(fily,label='filtered')
    plt.plot(za)
    plt.legend(loc = 'lower left')
    plt.show()

if __name__ == '__main__':
    rospy.init_node('KF', anonymous=True)
    run()