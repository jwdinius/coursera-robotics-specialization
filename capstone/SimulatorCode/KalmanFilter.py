#!/usr/bin/python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib import patches
#import pylab
import time
import math
from geom_utils import tag_pos, robot_pos

class KalmanFilter:
    """
    Class to keep track of the estimate of the robots current state using the
    Kalman Filter
    """
    def __init__(self, markers):
        """
        Initialize all necessary components for Kalman Filter, using the
        markers (AprilTags) as the map
        Input: 
        markers - an N by 4 array loaded from the parameters, with each element
            consisting of (x,y,theta,id) where x,y gives the 2D position of a
            marker/AprilTag, theta gives its orientation, and id gives its
            unique id to identify which one you are seeing at any given
            moment
        """
        self.markers = markers
        self.last_time = None # Used to keep track of time between measurements 
        self.Q_t = np.identity(2)
        self.R_t = np.identity(3)
        # YOUR CODE HERE
        self.x_t = np.zeros((3,1))
        self.P_t = 1000. * np.identity(3)

    def prediction(self, v, dt):
        """
        Performs the prediction step on the state x_t and covariance P_t
        Inputs:
        v - a number representing in m/s the commanded speed of the robot
        imu_meas - a 5 by 1 numpy array consistening of the values
            (acc_x,acc_y,acc_z,omega,time), with the fourth of the values giving
            the gyroscope measurement for angular velocity (which you should
            use as ground truth) and time giving the current timestamp. Ignore
            the first three values (they are for the linear acceleration which
            we don't use)
        Outputs: a tuple with two elements
        predicted_state - a 3 by 1 numpy array of the predction of the state
        predicted_covariance - a 3 by 3 numpy array of the predction of the
            covariance
        """
        # YOUR CODE HERE
        
        # x(k) = f(x(k-1),u(k-1))
        xk = self.x_t[0] + v[0]*np.cos(self.x_t[2])*dt
        yk = self.x_t[1] + v[0]*np.sin(self.x_t[2])*dt
        tk = self.x_t[2] + v[1]*dt

        # predict state
        self.x_t = np.array([[xk],[yk],[tk]]).reshape((3,1))

        # jacobian of f wrt x
        Fx = np.identity(3)

        # jacobian of f wrt u
        Fu      = np.zeros((3,2))
        Fu[0,0] = np.cos(self.x_t[2])*dt
        Fu[1,0] = np.sin(self.x_t[2])*dt
        Fu[2,1] = dt
        
        # predict covariance
        self.P_t = np.dot(Fx, np.dot(self.P_t, np.transpose(Fx))) + np.dot(Fu, np.dot(self.Q_t, np.transpose(Fu)))

        return


    def update(self,z_t):
        """
        Performs the update step on the state x_t and covariance P_t
        Inputs:
        z_t - an array of length N with elements that are 4 by 1 numpy arrays.
            Each element has the same form as the markers, (x,y,theta,id), with
            x,y gives the 2D position of the measurement with respect to the
            robot, theta the orientation of the marker with respect to the
            robot, and the unique id of the marker, which you can find the
            corresponding marker from your map
        Outputs:
        predicted_state - a 3 by 1 numpy array of the updated state
        predicted_covariance - a 3 by 3 numpy array of the updated covariance
        """
        # YOUR CODE HERE
        H = np.identity(3)
        S = np.add(np.dot(H,np.dot(self.P_t, np.transpose(H))), self.R_t)
        K = np.dot(self.P_t, np.dot(np.transpose(H),np.linalg.inv(S)))

        if z_t != None and z_t.any():
            for z in z_t:
                tag_w_pose = tag_pos(self.markers, z[3])
                rob_r_pose = robot_pos(tag_w_pose, z[:3])
                res        = np.subtract(rob_r_pose, np.dot(H, self.x_t))
                self.x_t   = np.add(self.x_t, np.dot(K, res))
                self.P_t   = np.dot(np.subtract(np.identity(3), np.dot(K, H)), self.P_t)

        return
        

    def step_filter(self, v, imu_meas, z_t):
        """
        Perform step in filter, called every iteration (on robot, at 60Hz)
        Inputs:
        v, imu_meas - descriptions in prediction. Will be None value if
            values are not available
        z_t - description in update. Will be None value if measurement is not
            available
        Outputs:
        x_t - current estimate of the state
        """
        # YOUR CODE HERE
        if imu_meas != None:
            if self.last_time == None:
                # initialize the Kalman filter with the measured position
                self.update(z_t)
                self.last_time = imu_meas[4, 0]
            else:
                time = imu_meas[4, 0]
                dt   = time - self.last_time
                self.prediction(v, dt)
                self.update(z_t)
                self.last_time = time

        return self.x_t
