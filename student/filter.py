# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

dt=params.dt
dq=params.q

class Filter:
    '''Kalman filter class'''
    def __init__(self):
        pass

    def F(self):
        ############
        # TODO Step 1: implement and return system matrix F
        print("----------->",params.dim_state) #6
        F=np.matrix([[1,0,0,dt,0,0],
                     [0,1,0,0,dt,0],
                     [0,0,1,0,0,dt],
                     [0,0,0,1,0,0],
                     [0,0,0,0,1,0],
                     [0,0,0,0,0,1]])

        return F
        
        ############
        # END student code
        ############ 

    def Q(self):
        ############
        # TODO Step 1: implement and return process noise covariance Q
        Q=np.zeros((6,6))
        q1 = ((dt**4)/4) * dq 
        q2 = ((dt**3)/3) * dq 
        q3 = ((dt**2)/2) * dq 
        q4 =   dt * dq 
        ############
        
        Q[0,0]= Q[1,1]=Q[2,2]=q1
        Q[0, 3] = Q[1, 4] = Q[2, 5] = q2
        Q[3, 0] = Q[4, 1] = Q[5, 2] = q3
        Q[3, 3] = Q[4, 4] = Q[5, 5] = q4
        ############

        return Q
        
        ############
        # END student code
        ############ 

    def predict(self, track):
        print(track, type(track), "this is track")

        ############
        # TODO Step 1: predict state x and estimation error covariance P to next timestep, save x and P in track
        ############
        x=self.F()*track.x
        P=(self.F()) *track.P* (self.F().transpose()) +self.Q()
        track.set_x(x)
        track.set_P(P)
        
        ############
        # END student code
        ############ 

    def update(self, track, meas):
        ############
        # TODO Step 1: update state x and covariance P with associated measurement, save x and P in track
        H=meas.sensor.get_H(track.x)
        gamma=self.gamma(track, meas)
        S=self.S(track, meas, H)
        K=track.P * H.transpose() * S.I
        x=track.x+(K*gamma)
        P=(np.eye(6)-K*H) * track.P
        track.set_P(P)
        track.set_x(x)
        

        ############
        
        ############
        # END student code
        ############ 
        track.update_attributes(meas)
    
    def gamma(self, track, meas):
        ############
        # TODO Step 1: calculate and return residual gamma
        gam=meas.z-meas.sensor.get_hx(track.x)
        
        ############

        return gam
        
        ############
        # END student code
        ############ 

    def S(self, track, meas, H):
        ############
        # TODO Step 1: calculate and return covariance of residual S
        s=H* track.P * H.transpose() + meas.R
        ############

        return s
        
        ############
        # END student code
        ############ 