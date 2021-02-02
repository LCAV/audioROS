#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
geometry.py: Some geometry functions useful across all simulations.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Pose, Point

from audio_interfaces_py.messages import get_quaternion

ROOM_DIM = np.array([10., 7., 5.])  # room dimensions [m].
SOURCE_POS = np.array([5., 6., 1.])  # source position [m], None for no external source.
STARTING_POS = np.array([5., 0.2, 1.]) # drone starting position [m]
YAW_DEG = 30 # starting absoltue yaw angle in degrees

def get_starting_pose():
    msg = Pose()
    msg.orientation = get_quaternion(YAW_DEG)
    msg.position = Point(
        x=STARTING_POS[0],
        y=STARTING_POS[1],
        z=STARTING_POS[2]
    )
    return msg


def global_positions(local_positions_2D, msg_pose, z=0):
    """
    Calculate global postions (e.g. of mics) based on the current pose (rotation and translation). 

    :param local_positions_2D: coordinates (e.g. of mics) in local coordinates (n_mics x 2)
    :param msg_pose: Pose message
    :param z: height in (e.g. of mics) in local coordinates 

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
    local_positions_3D = np.c_[local_positions_2D, np.full((n_pos, 1), z)]
    global_positions = rot.apply(local_positions_3D) + translation
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
