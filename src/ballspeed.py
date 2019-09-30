#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist 
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
ball = Pose()

time = 0
xo = 0
yo = 0
told = 0
tw = Twist()
bpub = rospy.Publisher('balltwist',Twist,queue_size = 10)

def tick(msg):
    global time
    time = msg.data

aa = rospy.Publisher('xpseed',Float64,queue_size=10)

def bp(msg):
    global ball
    global time
    global xo
    global yo
    global told
    global bpub
    global tw
    global aa
    ball  = msg
    dt = (time - told)/1000.0
    x = ball.position.x
    y = ball.position.y
    difx = x - xo
    dify = y - yo
    if dt != 0:
        vx = (x - xo)/dt
        vy = (y - yo)/dt
        tw.linear.x = vx
        tw.linear.y = vy
        aa.publish(vx)
    bpub.publish(tw)
    xo = x
    yo = y 
    told = time


def run():
    global ball
    
    rospy.init_node('ballspeed', anonymous=True)
    rospy.Subscriber('ballpose',Pose,bp)
    rospy.Subscriber('Time',Float64,tick)
    rospy.spin()
        

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass