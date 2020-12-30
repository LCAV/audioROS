#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
geometry.py: Some geometry functions useful across all simulations.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R


from audio_interfaces.msg import PoseRaw


def global_positions(local_positions_2D, msg_pose, z=0):
    """
    Calculate global postions (e.g. of mics) based on the current pose (rotation and translation). 

    :param local_positions_2D: coordinates (e.g. of mics) in local coordinates (n_mics x 2)
    :param msg_pose: Pose message 

    :return: global positions (n_mics x 3)
    """
    rotation = np.array([
        msg_pose.orientation.x, 
        msg_pose.orientation.y, 
        msg_pose.orientation.z, 
        msg_pose.orientation.w
    ])
    translation = np.array([
        msg_pose.position.x, 
        msg_pose.position.y, 
        msg_pose.position.z
    ])
    n_pos = local_positions_2D.shape[0]

    rot = R.from_quat(rotation)
    global_positions = np.empty((n_pos, 3), dtype=float)
    global_positions[:, :2] = local_positions_2D
    global_positions[:, 2] = z
    for i in range(n_pos):
        global_positions[i, :] = rot.apply(global_positions[i, :] - translation) + translation
    return global_positions


def get_relative_movement(pose1, pose2):
    """ Get the step length (in m) and rotation (in radiants) 
    between pose1 and pose2.

    """
    step_length = np.linalg.norm([
        pose2.position.x - pose1.position.x,
        pose2.position.y - pose1.position.y,
        pose2.position.z - pose1.position.z
    ])
    r1, r2 = [R.from_quat([p.orientation.x,
                           p.orientation.y,
                           p.orientation.z,
                           p.orientation.w]) for p in [pose1, pose2]]
    r = r2 * r1.inv() # "get angle2 - angle1"
    rotation = r.magnitude() # magnitude of rotation, in radiants
    return [step_length, rotation]
