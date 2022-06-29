"""
experiments.py: Some experiment variables for crazyflie.
"""

import numpy as np

# corresponds to the setup in BC325 with stepper motor:
WALL_ANGLE_DEG_STEPPER = 132  # angle of wall in degrees, at start of experiment
WALL_DISTANCE_CM_STEPPER = 8  # distance of wall in centimeters, at start of experiment
DISTANCES_CM = np.arange(50, step=1)[::-1] + WALL_DISTANCE_CM_STEPPER

# corresponds to flying experiments:
WALL_DISTANCE_M = 0.2  # distance of wall in meters, at start of experiment
WALL_ANGLE_DEG = 90  # angle of wall in degrees, at start of experiment

# used for anchoring Pose message in gateway, currently not used.
STARTING_POS = np.array([5.0, 0.2, 1.0])  # drone starting position [m]
STARTING_YAW_DEG = 0  # starting absoltue yaw angle in degrees
