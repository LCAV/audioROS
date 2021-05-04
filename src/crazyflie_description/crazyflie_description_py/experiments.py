#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
experiments.py: Some experiment variables
"""

import numpy as np

from audio_interfaces_py.messages import create_pose_message

ROOM_DIM = np.array([10.0, 7.0, 5.0])  # room dimensions [m].
SPEAKER_POSITION = np.array(
    [5.0, 6.0, 1.0]
)  # external source position [m], None for no external source.
STARTING_POS = np.array([5.0, 0.2, 1.0])  # drone starting position [m]
STARTING_YAW_DEG = 0  # starting absoltue yaw angle in degrees

WALL_DISTANCE_M = 0.2  # distance of wall in meters
WALL_ANGLE_DEG = -30  # angle of wall in degrees


def get_starting_pose_msg(timestamp=None):
    msg = create_pose_message(
        x=STARTING_POS[0],
        y=STARTING_POS[1],
        z=STARTING_POS[2],
        yaw_deg=STARTING_YAW_DEG,
        timestamp=timestamp,
    )
    return msg
