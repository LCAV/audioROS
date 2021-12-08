#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
wall_backend.py: Estiamte plane and pose information using factor graphs
"""

import itertools

import numpy as np
import scipy

import gtsam

X = gtsam.symbol_shorthand.X 
P = gtsam.symbol_shorthand.P 

#METHOD = "ISAM"
#METHOD = "GN"
EPS = 1e-10

def rad(angle_deg):
    return angle_deg / 180 * np.pi

def deg(angle_rad):
    return angle_rad * 180 / np.pi

def get_vector(azimuth, elevation, degrees=False):
    if degrees:
        azimuth *= np.pi / 180
        elevation *= np.pi / 180
    return np.r_[
        np.cos(azimuth) * np.cos(elevation), 
        np.sin(azimuth) * np.cos(elevation),
        np.sin(elevation)
    ]

def get_estimates(values, prob, sort=True, n_estimates=None):
    indices, __ = scipy.signal.find_peaks(prob)
    estimates = values[indices]
    prob_estimates = prob[indices]
    sort_indices = prob_estimates.argsort()[::-1]
    if sort:
        estimates = estimates[sort_indices]
        prob_estimates = prob_estimates[sort_indices]
    stds = (values[1]-values[0]) / (prob_estimates * np.sqrt(2 * np.pi))
    if n_estimates is None:
        return estimates, stds
    else:
        return estimates[:n_estimates], stds[:n_estimates]

class WallBackend(object):
    """ Node to map out the walls of a room using audio signals  """
    PARAMS_DICT = {}
    
    def __init__(self, verbose=False):
        # initialize ISAM
        params = gtsam.ISAM2Params()
        params.setRelinearizeSkip(1)
        self.isam = gtsam.ISAM2(params=params)
        self.result = None

        self.set_noise()

        self.pose_index = -1
        self.plane_index = 0

        self.verbose=verbose
        self.all_initial_estimates = gtsam.Values()

    def set_noise(self, 
            plane_azimuth_deg=EPS,
            plane_elevation_deg=EPS,
            plane_distance_cm=2,
            pose_roll_deg=EPS,
            pose_pitch_deg=EPS,
            pose_yaw_deg=EPS,
            pose_x_cm=EPS,
            pose_y_cm=EPS,
            pose_z_cm=EPS,
            ):
        # below are verified with unit tests tests/test_NoiseOrder.py
        self.plane_noise = gtsam.noiseModel.Diagonal.Sigmas([
            rad(plane_azimuth_deg), 
            rad(plane_elevation_deg), 
            plane_distance_cm*1e-2,
        ])
        self.pose_noise = gtsam.noiseModel.Diagonal.Sigmas([
            rad(pose_roll_deg),
            rad(pose_pitch_deg),
            rad(pose_yaw_deg),
            pose_x_cm*1e-2,
            pose_y_cm*1e-2,
            pose_z_cm*1e-2,
        ])

    def add_plane(self, distance, azimuth, elevation=0.0, plane_noise=None):
        if plane_noise is None:
            plane_noise = self.plane_noise
        else:
            plane_noise = gtsam.noiseModel.Diagonal.Sigmas(plane_noise)

        new_factors = gtsam.NonlinearFactorGraph()
        initial_estimates = gtsam.Values()

        vector = get_vector(azimuth=azimuth, elevation=elevation)
        current_plane = gtsam.OrientedPlane3(gtsam.Unit3(vector), distance)

        if (self.result is None) or not self.result.exists(P(self.plane_index)):
            if self.verbose:
                print("initial estimate", current_plane.planeCoefficients())
            initial_estimates.insert(P(self.plane_index), current_plane)
            self.all_initial_estimates.insert(P(self.plane_index), current_plane)

        if self.verbose:
            print("plane factor", current_plane.planeCoefficients())
        factor = gtsam.OrientedPlane3Factor(current_plane.planeCoefficients(), 
                                            plane_noise, 
                                            X(self.pose_index), P(self.plane_index))
        new_factors.push_back(factor)
        self.isam.update(new_factors, initial_estimates)

        # TODO: figure out when we are next to a new plane
        # self.plane_index += 1
        return factor

    def add_planes_from_distance_distribution(self, dists_cm, probs, azimuth_deg=90, azimuth_deg_std=10, n_estimates=1, verbose=False):
        """ Add plane factor from distance distribution and angle measurement. 

        :param dists_cm: distances in centimeters
        :param probs: corresponding probabilities
        :param azimuth_deg: angle of wall normal, from current pose.
        """

        distance_estimates, distance_stds = get_estimates(dists_cm, probs, n_estimates=n_estimates)
        angle_estimates = [rad(azimuth_deg)] 
        angle_stds = [rad(azimuth_deg_std)]

        if verbose:
            print("distance estimates:", distance_estimates)

        for d_i, a_i in itertools.product(range(len(distance_estimates)), range(len(angle_estimates))):
            distance = distance_estimates[d_i]
            azimuth = angle_estimates[a_i]
            #noise = [distance_stds[d_i], angle_stds[a_i], 0.0]

            wall_angle = azimuth - np.pi
            #print(f"INSIDE: add wall estimate at {distance:.1f}cm {deg(wall_angle):.0f}deg")
            # use the classes plane noise!
            self.add_plane(distance*1e-2, wall_angle, plane_noise=None)
            self.get_results()

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
            self.all_initial_estimates.insert(X(self.pose_index), current_pose)

        factor = gtsam.PriorFactorPose3(X(self.pose_index), current_pose, self.pose_noise)
        new_factors.push_back(factor)
        self.isam.update(new_factors, initial_estimates)
        return factor

    def get_results(self):
        self.result = self.isam.calculateEstimate()
        planes = gtsam.utilities.allOrientedPlane3s(self.result)
        poses = gtsam.utilities.allPose3s(self.result) 
        return planes, poses
