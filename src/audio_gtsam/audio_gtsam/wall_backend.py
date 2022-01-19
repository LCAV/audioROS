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

EPS = 1e-10

WALL_THRESH_M = 0.5  # difference of normal points of walls
USE_ISAM = True

N_VELOCITY_ESTIMATE = 3  # how many points to use for velocity estimate
DISTANCE_THRESHOLD_M = 0.2  # when to consider wall too close

LIMIT_DISTANCE_CM = 20  # threshold for adding wall to factor graph


def angle_difference(a1, a2):
    # return smallest angle from a1 to a2, signed.
    diff = a1 - a2
    return (diff + 180) % 360 - 180


def rad(angle_deg):
    return angle_deg / 180 * np.pi % (2 * np.pi)


def deg(angle_rad):
    return angle_rad * 180 / np.pi % 360


def plane_error(current_plane, new_plane):
    """ Calculate the Euclidean distance between normal points of two planes. 

    This function is used to determined when to consider the new_plane measurement as a new factor, 
    instead of associating it with current_plane.

    """
    current_point = -current_plane.normal().point3() * current_plane.distance()
    new_point = -new_plane.normal().point3() * new_plane.distance()
    return np.linalg.norm(new_point - current_point)


def get_vector(azimuth, elevation, degrees=False):
    if degrees:
        azimuth *= np.pi / 180
        elevation *= np.pi / 180
    return np.r_[
        np.cos(azimuth) * np.cos(elevation),
        np.sin(azimuth) * np.cos(elevation),
        np.sin(elevation),
    ]


def get_angle(vector):
    normal_vector = vector / np.linalg.norm(
        vector
    )  # just in case it's not already norm-one
    return np.arctan2(normal_vector[1], normal_vector[0])


def get_azimuth_angle(normal_vector, degrees=False):
    angle_rad = get_angle(normal_vector)
    angle_rad += np.pi  # plus pi because of different conventions
    return angle_rad * 180 / np.pi if degrees else angle_rad


def get_estimates(values, prob, sort=True, n_estimates=None):
    indices, __ = scipy.signal.find_peaks(prob)
    estimates = values[indices]
    prob_estimates = prob[indices]
    sort_indices = prob_estimates.argsort()[::-1]
    if sort:
        estimates = estimates[sort_indices]
        prob_estimates = prob_estimates[sort_indices]
    stds = (values[1] - values[0]) / (prob_estimates * np.sqrt(2 * np.pi))
    if n_estimates is None:
        return estimates, stds
    else:
        return estimates[:n_estimates], stds[:n_estimates]


class WallBackend(object):
    """ Node to map out the walls of a room using audio signals  """

    PARAMS_DICT = {}

    def __init__(self, use_isam=USE_ISAM):
        self.use_isam = use_isam
        if self.use_isam:
            params = gtsam.ISAM2Params()
            params.setRelinearizeSkip(1)
            self.isam = gtsam.ISAM2(params=params)
        else:
            self.graph = gtsam.NonlinearFactorGraph()

        self.result = None

        self.set_confidence()

        self.pose_index = -1
        self.plane_index = -1

        self.all_initial_estimates = gtsam.Values()

        self.need_to_update_results = True

    def set_confidence(
        self,
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
        self.plane_noise = gtsam.noiseModel.Diagonal.Sigmas(
            [
                rad(plane_azimuth_deg),
                rad(plane_elevation_deg),
                plane_distance_cm * 1e-2,
            ]
        )
        self.pose_noise = gtsam.noiseModel.Diagonal.Sigmas(
            [
                rad(pose_roll_deg),
                rad(pose_pitch_deg),
                rad(pose_yaw_deg),
                pose_x_cm * 1e-2,
                pose_y_cm * 1e-2,
                pose_z_cm * 1e-2,
            ]
        )

    def add_plane(
        self, distance, azimuth, elevation=0.0, plane_noise=None, verbose=False
    ):
        if self.need_to_update_results:
            self.get_results()

        if plane_noise is None:
            plane_noise = self.plane_noise
        else:
            plane_noise = gtsam.noiseModel.Diagonal.Sigmas(plane_noise)

        new_factors = gtsam.NonlinearFactorGraph()
        initial_estimates = gtsam.Values()

        vector = get_vector(azimuth=azimuth, elevation=elevation)
        new_plane = gtsam.OrientedPlane3(gtsam.Unit3(vector), distance)

        # Figure out when we are next to a new plane
        current_plane_global = None
        try:
            current_plane_global = self.result.atOrientedPlane3(P(self.plane_index))
        except:
            pass

        if current_plane_global is not None:
            current_pose = self.result.atPose3(X(self.pose_index))
            current_plane = current_plane_global.transform(current_pose)
            error = plane_error(current_plane, new_plane)

            # error = np.linalg.norm(current_plane.errorVector(new_plane))
            # if verbose:
            #    print(f"current plane: {get_azimuth_angle(current_plane.normal().point3(), degrees=True):.0f}, {current_plane.distance():.2f}m")
            #    print(f"new plane: {get_azimuth_angle(new_plane.normal().point3(), degrees=True):.0f}, {new_plane.distance():.2f}m")
            #    print(f"plane error: {error:.2f}")

            if error > WALL_THRESH_M:
                print("adding new plane!")
                self.plane_index += 1
            else:
                # print("not adding new plane, match error:", error)
                pass
        else:
            self.plane_index += 1

        if (self.result is None) or not self.result.exists(P(self.plane_index)):
            initial_estimates.insert(P(self.plane_index), new_plane)
            self.all_initial_estimates.insert(P(self.plane_index), new_plane)

        if verbose:
            azimuth = get_azimuth_angle(new_plane.normal().point3(), degrees=True)
            distance = new_plane.distance()
            print(
                f"add to plane {self.plane_index} and pose {self.pose_index}: {azimuth:.0f}deg, {distance:.2f}m"
            )
            current_pose = self.result.atPose3(X(self.pose_index))
            new_plane_global = new_plane.transform(current_pose.inverse())
            azimuth = get_azimuth_angle(
                new_plane_global.normal().point3(), degrees=True
            )
            distance = new_plane_global.distance()
            print(
                f"add to plane (global){self.plane_index}: {azimuth:.0f}deg, {distance:.2f}m\n"
            )

        factor = gtsam.OrientedPlane3Factor(
            new_plane.planeCoefficients(),
            plane_noise,
            X(self.pose_index),
            P(self.plane_index),
        )
        new_factors.push_back(factor)
        if self.use_isam:
            self.isam.update(new_factors, initial_estimates)
        else:
            self.graph.add(factor)

        self.need_to_update_results = True
        return factor

    def add_pose(self, r_world, yaw, verbose=False):
        self.pose_index += 1
        new_factors = gtsam.NonlinearFactorGraph()
        initial_estimates = gtsam.Values()

        current_pose = gtsam.Pose3(
            r=gtsam.Rot3.Ypr(yaw, 0, 0), t=gtsam.Point3(*r_world)
        )
        if (self.result is None) or not self.result.exists(X(self.pose_index)):
            if verbose:
                print(
                    "initial estimate",
                    current_pose.translation(),
                    current_pose.rotation().yaw(),
                )
            initial_estimates.insert(X(self.pose_index), current_pose)
            self.all_initial_estimates.insert(X(self.pose_index), current_pose)

        factor = gtsam.PriorFactorPose3(
            X(self.pose_index), current_pose, self.pose_noise
        )
        new_factors.push_back(factor)
        if self.use_isam:
            self.isam.update(new_factors, initial_estimates)
        else:
            self.graph.add(factor)

        self.need_to_update_results = True
        return factor

    def add_planes_from_distributions(
        self,
        dists_cm,
        probs,
        azimuths_deg,
        probs_angles,
        n_estimates=1,
        limit_distance=LIMIT_DISTANCE_CM,
        verbose=False,
    ):
        """ Add plane factor from distance and angle distributions.
        """

        # TODO(FD) can remove n_estimates parameter because factor graph
        # does not deal well with multiple measurements.
        distance_estimates, distance_stds = get_estimates(
            dists_cm, probs, n_estimates=n_estimates
        )
        angle_estimates, angle_stds = get_estimates(
            azimuths_deg, probs_angles, n_estimates=n_estimates
        )

        # if verbose:
        #    print("distance estimates:", distance_estimates)

        for d_i, a_i in itertools.product(
            range(len(distance_estimates)), range(len(angle_estimates))
        ):
            distance = distance_estimates[d_i]
            azimuth = angle_estimates[a_i]

            if distance > limit_distance:
                continue

            # because the angle estimates are very unreliable we fall back to
            # a wall exactly ahead of us, should the estimate say something
            # completely off.
            v_estimate_global = self.get_velocity_estimate()
            if v_estimate_global is not None:
                normal_vector_global = -v_estimate_global / np.linalg.norm(
                    v_estimate_global
                )

                # hack to get the local normal vector
                # TODO(FD) in add_plane, we will convert the angle back to normal, so should
                # probably refactor this. Also, when we don't care about the distance it
                # is proably much easier to get the normal vector.
                plane_global = gtsam.OrientedPlane3(
                    np.r_[normal_vector_global, 0, 10]
                )  # 10 is distance, doesn't matter

                current_pose = self.result.atPose3(X(self.pose_index))
                plane_local = plane_global.transform(current_pose)
                azimuth_estimated = deg(
                    get_azimuth_angle(plane_local.normal().point3())
                )
                print(f"  wall ahead of us: {azimuth_estimated:.0f}")
                print(f"  wall estimate: {azimuth:.0f}")

                if abs(angle_difference(azimuth, azimuth_estimated)) > 30:
                    if verbose:
                        print(f"  replacing with wall ahead of us")
                    azimuth = azimuth_estimated
                # else:
                # print(f"ok angle estimate, using {azimuth} instead of {azimuth_estimated}")

            # because normal points in opposite direction than what we defined for the azimuth angle.
            wall_angle_rad = rad(azimuth) + np.pi
            self.add_plane(
                distance * 1e-2, wall_angle_rad, plane_noise=None, verbose=verbose
            )

    def add_plane_from_distances(
        self,
        dists_cm,
        probs,
        n_estimates=1,
        limit_distance=LIMIT_DISTANCE_CM,
        verbose=False,
    ):
        """ 
        Add plane factor from distance distributions, using angle ahead.
        """
        distances, distance_stds = get_estimates(dists_cm, probs, n_estimates=1)
        if not len(distances):
            return

        distance = distances[0]
        if distance > limit_distance:
            return

        # because the angle estimates are very unreliable we fall back to
        # a wall exactly ahead of us, should the estimate say something
        # completely off.
        v_estimate_global = self.get_velocity_estimate()
        if v_estimate_global is None:
            return

        normal_vector_global = -v_estimate_global / np.linalg.norm(v_estimate_global)
        # hack to get the local normal vector
        # TODO(FD) in add_plane, we will convert the angle back to normal, so should
        # probably refactor this. Also, when we don't care about the distance it
        # is proably much easier to get the normal vector.
        plane_global = gtsam.OrientedPlane3(
            np.r_[normal_vector_global, 0, 10]
        )  # 10 is distance, doesn't matter

        current_pose = self.result.atPose3(X(self.pose_index))
        plane_local = plane_global.transform(current_pose)
        azimuth_estimated_rad = get_azimuth_angle(plane_local.normal().point3())
        wall_angle_rad = azimuth_estimated_rad + np.pi
        self.add_plane(
            distance * 1e-2, wall_angle_rad, plane_noise=None, verbose=verbose
        )

    def get_results(self):
        if self.use_isam:
            self.result = self.isam.calculateEstimate()
        else:
            optimizer = gtsam.LevenbergMarquardtOptimizer(
                self.graph, self.all_initial_estimates
            )
            self.result = optimizer.optimizeSafely()
        self.need_to_update_results = False
        return None, None
        # planes = gtsam.utilities.allOrientedPlane3s(self.result)
        # poses = gtsam.utilities.allPose3s(self.result)
        # return planes, poses

    def check_wall(self, verbose=False):
        # no wall added yet
        if self.plane_index < 0:
            return False

        if self.need_to_update_results:
            self.get_results()

        v_estimate = self.get_velocity_estimate()
        if v_estimate is None:
            return False

        plane_estimate = self.result.atOrientedPlane3(
            P(self.plane_index)
        )  # in global coordinates!
        normal = plane_estimate.normal().point3()

        distance_estimate = abs(self.get_distance_estimate())
        if (distance_estimate < DISTANCE_THRESHOLD_M) & (v_estimate.dot(normal) < 0):
            if verbose:
                v_angle = np.arctan2(v_estimate[1], v_estimate[0]) * 180 / np.pi
                wall_angle = np.arctan2(normal[1], normal[0]) * 180 / np.pi
                print(
                    f"At pose {self.pose_index}, plane {self.plane_index}: turn around! velocity angle: {v_angle:.0f}deg, wall normal angle:{wall_angle:.0f}deg, distance:{distance_estimate*1e2:.0f}cm"
                )
            return True
        return False

    def get_velocity_estimate(self):
        """ Return the direction estimate (not true velocity because we don't save timing) """
        if self.pose_index < 1:  # need at least two poses to calculate velocity
            return None

        if self.need_to_update_results:
            self.get_results()

        n_poses = min(self.pose_index, N_VELOCITY_ESTIMATE)
        xs = np.empty((n_poses, 3))
        for i in range(n_poses):
            idx = self.pose_index - n_poses + i
            xs[i, :] = self.result.atPose3(X(idx)).translation()
        return np.mean(np.diff(xs, axis=0), axis=0)  # in global coordinates!

    def get_distance_estimate(self):
        if (self.pose_index < 0) or (self.plane_index < 0):
            return None

        if self.need_to_update_results:
            self.get_results()

        plane_estimate = self.result.atOrientedPlane3(P(self.plane_index))
        latest_pose_estimate = self.result.atPose3(X(self.pose_index))
        plane_estimate_from_pose = plane_estimate.transform(latest_pose_estimate)
        return plane_estimate_from_pose.distance()

    def get_angle_estimate(self):
        if self.plane_index < 0:
            return None

        plane_estimate = self.result.atOrientedPlane3(
            P(self.plane_index)
        )  # in global coordinates!
        normal = plane_estimate.normal().point3()
        return get_azimuth_angle(normal, degrees=True)
