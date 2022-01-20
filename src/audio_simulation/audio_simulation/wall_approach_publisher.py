"""
wall_approach_publisher.py: Approach wall inside a room. 
"""

import numpy as np
import rclpy

from audio_simulation.linear_pose_publisher import LinearPosePublisher, EPS
from audio_simulation.geometry import ROOM_DIM

VELOCITY = np.array([0.0, 0.02, 0.0])  # m/s, in drone coordinates
POSITION = np.array([ROOM_DIM[0] / 2.0, EPS, ROOM_DIM[2] / 2.0])
MAX_Y = 0.5  # in m


def main(args=None):
    rclpy.init(args=args)

    linear_pub = LinearPosePublisher(
        constant_velocity=VELOCITY,
        starting_position=POSITION,
        starting_yaw_deg=0,
        max_y=MAX_Y,
    )

    rclpy.spin(linear_pub)

    linear_pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
