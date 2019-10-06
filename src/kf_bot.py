#!/usr/bin/env python
import rospy
import numpy as np 
import time
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose 
from std_msgs.msg import Float64,Int32

time = 0
zx = np.array([0,0])
zy = np.array([0,0])
zw = 0.0
zx_com = np.array([0.0,0.0])
zy_com = np.array([0.0,0.0])
zw_com = 0.0

def tick(msg):
    global time
    time = msg.data

def bpose(msg):
    global zx
    global zy
    zx[0] = msg.position.x
    zy[0] = msg.position.y

def btwist(msg):
    global zx
    global zy
    global zw
    zx[1] = msg.linear.x
    zy[1] = msg.linear.y
    zw = msg.angular.z

def btwistcom(msg):
    global zx_com
    global zy_com
    global zw_com
    zx_com[1] = msg.linear.x
    zy_com[1] = msg.linear.y
    zw_com = msg.angular.z

class state:
    def __init__(self,P,Q,R):
        global time
        self.R= R
        self.P= P
        self.Q=Q
        self.dt=0.0
        self.time=0.0
        self.to=0.0
        self.I = np.identity(2)
        self.A = np.array([[1,self.dt*0.7/1000.0],[0,0.7]])
        self.B = np.array([[1,self.dt*0.3/1000.0],[0,10000*0.3]])
        self.X_est = np.array([0,0])
        self.omega_est = 0.0
        #self.KG = self.P.dot(np.linalg.inv(self.P + self.R))

    def dim1_kf(self,z_w,zw_com):
        #prediction step
        x_pred = 0.8*self.omega_est + zw_com*0.2
        self.P = self.P + self.Q
        
        # update step
        residual = z_w - x_pred
        Z = self.R + self.P
        self.K = self.P/Z
        self.P = (1-self.K)*self.P
        self.omega_est = x_pred + self.K*residual   
        return self.omega_est


    def kalman_filter(self,z,z_com):
        global time
        #Kalman filter
        self.dt = time-self.to
        self.A = np.array([[1,self.dt*0.7/1000.0],[0,0.5]])
        self.B = np.array([[1,self.dt*0.3/1000.0],[0,10000/2]])
        self.Z = z
        self.U = z_com

        #prediction
        self.X_pred=self.A.dot(self.X_est) + self.B.dot(self.U)
        self.P=self.A.dot(self.P.dot((self.A).T)) + self.Q

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
    global zw
    global zx_com
    global zy_com
    P = np.array([[1,0],[0,1]])
    Q = P
    R = P + np.array([[0,0],[0,1]])
    x = state(P,Q,R)
    y = state(P,Q,R)
    w = state(0.1,0.001,0.01)
    rospy.Subscriber('bot1pose',Pose,bpose)
    rospy.Subscriber('Time',Float64,tick)
    rospy.Subscriber('bot1twistmeas',Twist,btwist)
    rospy.Subscriber('bot1twistglobal',Twist,btwistcom)
    a = rospy.Publisher('/xspeed',Float64,queue_size=10)
    b = rospy.Publisher('/yspeed',Float64,queue_size=10)
    c = rospy.Publisher('/omega',Float64,queue_size=10)
    rate = rospy.Rate(20)
    while(True):
        X = x.kalman_filter(zx,zx_com)
        Y = y.kalman_filter(zy,zy_com)
        W = w.dim1_kf(zw)
        print('pose',int(X[0]),int(Y[0]))
        print('twist',X[1],Y[1])
        print('omega measured',zw,'omega filtered',W)
        xs = Float64()
        ys = Float64()
        ws = Float64()
        xs.data = X[1]
        ys.data = Y[1]
        ws.data = W
        a.publish(xs)
        b.publish(ys)
        c.publish(ws)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('KF', anonymous=True)
    run()