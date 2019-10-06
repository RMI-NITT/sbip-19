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
zw = 0.0

zxcmd = 0.0
zycmd = 0.0
zwcmd = 0.0

filteredx = 0.0
filteredy = 0.0
filteredw = 0.0

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
    global zw
    zx[1] = msg.linear.x*1.0
    zy[1] = msg.linear.y*1.0
    zw = msg.angular.z*1.0


def btwistg(msg):
    global zxcmd
    global zycmd
    global zwcmd
    zxcmd = msg.linear.x*1.0
    zycmd = msg.linear.y*1.0
    zwcmd = msg.angular.z*1.0

def da(msg):
    global filteredx
    filteredx = msg.data

def db(msg):
    global filteredy
    filteredy = msg.data


def dc(msg):
    global filteredw
    filteredw = msg.data

def run():
    global zx
    global zy
    global zw
    global zxcmd
    global zycmd
    global zwcmd
    global filteredx
    global filteredy
    global filteredw
    rospy.Subscriber('bot1pose',Pose,bpose)
    rospy.Subscriber('Time',Float64,tick)
    rospy.Subscriber('bot1twistmeas',Twist,btwist)
    rospy.Subscriber('bot1twistglobal',Twist,btwistg)
    rospy.Subscriber('xspeed',Float64,da)
    rospy.Subscriber('yspeed',Float64,db)
    rospy.Subscriber('omega',Float64,dc)
    rate = rospy.Rate(20)
    ratio = []
    cmdx = []
    cmdy = []
    cmdw = []
    measx = []
    measy = []
    measw = []
    filx = []
    fily = []
    filw = []
    za = []
    i = 0
    while(i<100):
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
            cmdw.append(zwcmd*100)
            measw.append(zw)
            filw.append(filteredw)
            i = i+1
        rate.sleep()
    print 'done'

    plt.subplot(3, 1, 1)
    plt.plot(cmdx,label='command')
    plt.plot(measx,label='measured')
    plt.plot(filx,label='filtered')
    plt.plot(za)
    plt.legend(loc = 'lower left')
    plt.subplot(3, 1, 2)
    plt.plot(cmdy,label='command')
    plt.plot(measy,label='measured')
    plt.plot(fily,label='filtered')
    plt.plot(za)
    plt.legend(loc = 'lower left')
    plt.subplot(3, 1, 3)
    plt.plot(cmdw,label='command')
    plt.plot(measw,label='measured')
    plt.plot(filw,label='filtered')
    plt.plot(za)
    plt.legend(loc = 'lower left')
    plt.show()

if __name__ == '__main__':
    rospy.init_node('KF', anonymous=True)
    run()