#!/usr/bin/env python

from __future__ import print_function

import math
import roslib
import numpy as np
roslib.load_manifest('sbip')
import sys

X = np.matrix([[0],[0], [0]], dtype = float)
Y = np.matrix([[0],[0], [0]], dtype = float)
Px = np.eye(3, dtype = float)
Py = np.eye(3, dtype = float)
Q = np.eye(3, dtype = float)
Q[0,0] = 1
Q[1,1] = 120
Q[2,2] = 1440
del_t = 0.0

#Propogates present pose through n time instants of del_t each. Takes X,Y state vectors(3x1), Px, Py covariance matrices(3x3).
#Returns x_pred, y_pred, future_Px, future_Py of only position
def predict_pose(X, Y, Px, Py, del_t, n):
    
    A = np.matrix([[1, (n*del_t), (((n*del_t)**2)/2)],[0, 1, (n*del_t)], [0, 0, 1]], dtype=float)
    future_X = X
    future_Y = Y
    future_Px = Px[0,0]
    future_Py = Py[0,0]

    if n == -1:
        stop_t = math.sqrt(X.item(1)**2 + Y.item(1)**2)   / math.sqrt(X.item(2)**2 + Y.item(2)**2)
        stop_frame_count = int(stop_t/del_t) + 1
        n = stop_frame_count

    future_X = np.matmul(A, X)
    future_Px = np.matmul(np.matmul(np.transpose(A), Px),A).item(0) + n*Q[0,0]
    future_Y = np.matmul(A, Y)
    future_Py = np.matmul(np.matmul(np.transpose(A), Py),A).item(0) + n*Q[0,0]
    x_pred = future_X[0]
    y_pred = future_Y[0]

    return(x_pred, y_pred, future_Px, future_Py)