#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
constants.py: Some project-wide constants. 
"""
import numpy as np

SPEED_OF_SOUND = 343  # m/s
AUDIO_SAMPLING_RATE = 44100  # Hz, for soundcard
PLATFORM = "crazyflie"
#PLATFORM = "epuck"

ROOM_DIM = np.array([10.0, 7.0, 5.0])  # room dimensions [m].
SPEAKER_POSITION = np.array([10.0, 3.0, 1.0])  # external source position [m], None for no external source.
STARTING_POS = np.array([5.0, 0.2, 1.0])  # drone starting position [m]
STARTING_YAW_DEG = 0  # starting absoltue yaw angle in degrees
