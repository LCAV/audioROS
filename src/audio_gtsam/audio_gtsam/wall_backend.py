#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
wall_backend.py: 
"""

import numpy as np
import gtsam

X = gtsam.symbol_shorthand.X 
P = gtsam.symbol_shorthand.P 

def get_vector(azimuth, elevation, degrees=False):
    if degrees:
        azimuth *= np.pi / 180
        elevation *= np.pi / 180
    return np.r_[
        np.cos(azimuth) * np.cos(elevation), 
        np.sin(azimuth) * np.cos(elevation),
        np.sin(elevation)
    ]

class WallBackend(object):
    """ Node to map out the walls of a room using audio signals  """
    PARAMS_DICT = {}
    
    def __init__(self, verbose=False):
        # initialize ISAM
        params = gtsam.ISAM2Params()
        params.setRelinearizeSkip(1)
        self.isam = gtsam.ISAM2(params=params)
        self.result = None

        self.plane_noise = gtsam.noiseModel.Diagonal.Sigmas([0.8, 0.8, 5])
        self.pose_noise = gtsam.noiseModel.Isotropic.Sigma(6, 0.01)

        self.pose_index = -1
        self.plane_index = 0

        self.verbose=verbose

    def add_plane(self, distance, azimuth, elevation):
        new_factors = gtsam.NonlinearFactorGraph()
        initial_estimates = gtsam.Values()

        vector = get_vector(azimuth=azimuth, elevation=elevation)
        current_plane = gtsam.OrientedPlane3(gtsam.Unit3(vector), distance)

        if (self.result is None) or not self.result.exists(P(self.plane_index)):
            if self.verbose:
                print("initial estimate", current_plane.planeCoefficients())
            initial_estimates.insert(P(self.plane_index), current_plane)

        if self.verbose:
            print("plane factor", current_plane.planeCoefficients())
        factor = gtsam.OrientedPlane3Factor(current_plane.planeCoefficients(), 
                                            self.plane_noise, 
                                            X(self.pose_index), P(self.plane_index))
        new_factors.push_back(factor)
        self.isam.update(new_factors, initial_estimates)

        # TODO: figure out when we are next to a new plane
        # self.plane_index += 1

    def add_pose(self, r_world, yaw):
        self.pose_index += 1
        new_factors = gtsam.NonlinearFactorGraph()
        initial_estimates = gtsam.Values()

        current_pose = gtsam.Pose3(
                r=gtsam.Rot3.Ypr(yaw, 0, 0), 
                t=gtsam.Point3(*r_world)
        )
        if (self.result is None) or not self.result.exists(X(self.pose_index)):
            if self.verbose:
                print("initial estimate", current_pose.translation(), current_pose.rotation().yaw())
            initial_estimates.insert(X(self.pose_index), current_pose)

        factor = gtsam.PriorFactorPose3(X(self.pose_index), current_pose, self.pose_noise)
        new_factors.push_back(factor)
        self.isam.update(new_factors, initial_estimates)

    def get_results(self):
        self.result = self.isam.calculateEstimate()
        planes = gtsam.utilities.allOrientedPlane3s(self.result)
        poses = gtsam.utilities.allPose3s(self.result) 
        return planes, poses
