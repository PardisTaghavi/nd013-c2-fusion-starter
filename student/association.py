# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Data association class with single nearest neighbor association and gating based on Mahalanobis distance
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
from turtle import distance
import numpy as np
from scipy.stats.distributions import chi2
import math
# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

import misc.params as params 
gating_threshold=params.gating_threshold
class Association:
    '''Data association class with single nearest neighbor association and gating based on Mahalanobis distance'''
    def __init__(self):
        self.association_matrix = np.matrix([])
        self.unassigned_tracks = []
        self.unassigned_meas = []
        
    def associate(self, track_list, meas_list, KF):
             
        ############
        # TODO Step 3: association:
        # - replace association_matrix with the actual association matrix based on Mahalanobis distance (see below) for all tracks and all measurements
        # - update list of unassigned measurements and unassigned tracks
        ############
        N=len(track_list)
        M=len(meas_list)
        print(N,M,"N,M")
        # the following only works for at most one track and one measurement
        matrix1 = []#np.matrix([]) # reset matrix
        mat=[]
        if len(meas_list) > 0:
            self.unassigned_meas = np.arange(len(meas_list)).tolist()
            print("*******",len(self.unassigned_meas))
        if len(track_list) > 0:
            self.unassigned_tracks =np.arange(len(track_list)).tolist()
            print("*******2",len(self.unassigned_tracks),"\n")
            print("unasg", self.unassigned_tracks)


        if len(meas_list) > 0 and len(track_list) > 0: 
            print("flag-a")
            #print(meas_list.z, "pppp")
            for track in (track_list):
                mat=[]
                for meas in (meas_list):
                    distance=self.MHD(track, meas, KF)
                    #print(distance)
                    if self.gating(distance,  meas.sensor):
                        mat.append( distance)
                    else:
                        mat.append(np.inf)
                matrix1.append(mat)
            self.association_matrix=np.matrix(matrix1)
                   
                    
        print("\n")

        print(self.association_matrix, "self.association_matrix")
        #return self.association_matrix
        
        ############
        # END student code
        ############ 
                
    def get_closest_track_and_meas(self):
        ############
        # TODO Step 3: find closest track and measurement:
        # - find minimum entry in association matrix
        # - delete row and column
        # - remove corresponding track and measurement from unassigned_tracks and unassigned_meas
        # - return this track and measurement
        ############
        #A=self.association_matrix
        print(self.association_matrix.shape,"A.shapeeeee")
        if np.min(self.association_matrix)==np.inf:
            return np.nan, np.nan
        
        min_index=np.unravel_index(self.association_matrix.argmin(), self.association_matrix.shape)
        print("compare", len(self.unassigned_tracks), "|||",min_index)
        track_index=min_index[0]
        meas_index=min_index[1]
        self.association_matrix = np.delete(self.association_matrix, track_index, 0) 
        self.association_matrix= np.delete(self.association_matrix, meas_index, 1)


        # the following only works for at most one track and one measurement
        
        update_track = self.unassigned_tracks[track_index]
        update_meas = self.unassigned_meas[meas_index]
        
        # remove from list
        self.unassigned_tracks.remove(update_track) 
        self.unassigned_meas.remove(update_meas)
        #self.association_matrix = A
        ############
        # END student code
        ############ 
        return update_track, update_meas     

    def gating(self, MHD, sensor): 
        ############
        # TODO Step 3: return True if measurement lies inside gate, otherwise False
        ############
        
        lim=chi2.ppf(gating_threshold, df=sensor.dim_meas )
        if MHD < lim:
            return True
        else:
            return False
        
        ############
        # END student code
        ############ 
        



    def MHD(self, track, meas, KF):
        ############
        # TODO Step 3: calculate and return Mahalanobis distance
        ############
        H=meas.sensor.get_H(track.x)
        gm = KF.gamma(track, meas)
        S = KF.S(track, meas, H)
        MHD = math.sqrt(gm.transpose()*S.I*gm )# Mahalanobis distance formula
        print((MHD),"----MHd")
        ############
        # END student code
        ############
        
        return MHD

    
    def associate_and_update(self, manager, meas_list, KF):
        # associate measurements and tracks
        self.associate(manager.track_list, meas_list, KF)
    
        # update associated tracks with measurements
        print(manager.track_list, "manager.track_list- association")
        #print(self.association_matrix.shape,"self.association_matrix.shape")
        self.association_matrix=np.array(self.association_matrix)
        while self.association_matrix.shape[0]>0 and self.association_matrix.shape[1]>0:
            print("flag-c")
            # search for next association between a track and a measurement
            ind_track, ind_meas = self.get_closest_track_and_meas()
            if np.isnan(ind_track):
                print('---no more associations---')
                break
            track = manager.track_list[ind_track]
            
            # check visibility, only update tracks in fov    
            if not meas_list[0].sensor.in_fov(track.x):
                continue
            
            # Kalman update
            print('update track', track.id, 'with', meas_list[ind_meas].sensor.name, 'measurement', ind_meas)
            KF.update(track, meas_list[ind_meas])
            
            # update score and track state 
            manager.handle_updated_track(track)
            
            # save updated track
            manager.track_list[ind_track] = track
            
        # run track management 
        manager.manage_tracks(self.unassigned_tracks, self.unassigned_meas, meas_list)
        
        for track in manager.track_list:            
            print('track', track.id, 'score =', track.score)