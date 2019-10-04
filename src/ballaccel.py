#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist 
from std_msgs.msg import Float64
ball = Twist()

time = 0


vxo = 0
vyo = 0
told = 0
tw = Twist()
bpub = rospy.Publisher('ballaccela',Twist,queue_size = 10)
aa = rospy.Publisher('xaccel',Float64,queue_size=10)

def tick(msg):
    global time
    time = msg.data
    
def bt(msg):
    global bpub
    global vxo
    global vyo
    global tw
    global told
    global ball
    global aa
    ball  = msg
    global time
    dt = (time - told)/1000.0
    vx = ball.linear.x
    vy = ball.linear.y
    if dt != 0:
        ax = (vx-vxo)/dt
        ay = (vy-vyo)/dt
        tw.linear.x = ax
        tw.linear.y = ay
    #aa.publish(ax)
    bpub.publish(tw)
    vxo = vx
    vyo = vy 
    told = time

def run():
    global ball
    global time
    rospy.init_node('ballaccel', anonymous=True)
    rospy.Subscriber('balltwist',Twist,bt)
    rospy.Subscriber('Time',Float64,tick)
    rospy.spin()
        

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass