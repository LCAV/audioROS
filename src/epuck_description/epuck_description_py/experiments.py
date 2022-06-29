"""
experiments.py: Some experiment variables for epuck.
"""
import numpy as np

# corresponds to the setup in BC325 on the floor
WALL_ANGLE_DEG_STEPPER = 270  # angle of wall in degrees, at start of experiment
DISTANCES_CM = np.linspace(17.6, 60, 30)[::-1] # range of distances travelled in wall detection experiment
