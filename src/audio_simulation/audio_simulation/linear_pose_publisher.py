"""
linear_pose_publisher.py: Publish linear translational movement of Crazyflie inside a room. 
"""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import PoseStamped

from audio_interfaces_py.messages import (
    create_pose_message_from_arrays,
    create_pose_message,
)
from audio_simulation.geometry import (
    ROOM_DIM,
    STARTING_POS,
    STARTING_YAW_DEG,
)

VELOCITY = np.array([0.0, 0.5, 0.0])  # m/s, in drone coordinates
EPS = 0.2  # m, safety margin from walls
MAX_Y = None


class LinearPosePublisher(Node):
    def __init__(
        self,
        starting_position=STARTING_POS,
        constant_velocity=VELOCITY,
        starting_yaw_deg=STARTING_YAW_DEG,
        max_y=MAX_Y,
    ):
        super().__init__("linear_pose_publisher")

        self.publisher_pose = self.create_publisher(PoseStamped, "geometry/pose", 10)

        self.rot = R.from_euler("z", starting_yaw_deg, degrees=True)
        self.constant_velocity = constant_velocity
        self.position = starting_position
        self.max_y = max_y
        assert np.all(
            (EPS <= self.position) | (self.position <= ROOM_DIM - EPS)
        ), "starting position not inside room!"

        # start movement
        self.timer_period = 0.5  # seconds
        # we do not need to wait before starting the timer
        # because the timer first waits for self.timer_period.
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def get_time_ms(self):
        return int(round(time.time() * 1e3))

    def timer_callback(self):
        delta = self.rot.apply(self.timer_period * self.constant_velocity)
        new_position = self.position + delta

        # if we het the wall, revert direction
        if np.any(new_position > np.array(ROOM_DIM) - EPS) or np.any(
            new_position < EPS
        ):
            self.get_logger().warn("touched wall, turn direction")
            self.constant_velocity = -self.constant_velocity
            # self.rot = self.rot * R.from_euler('z', 180, degrees=True)
            new_position = self.position

        # if we reached max_y, revert direction.
        if (self.max_y is not None) and (new_position[1] > self.max_y):
            self.get_logger().warn(f"reached {self.max_y}, turn direction")
            self.constant_velocity = -self.constant_velocity
            # self.rot = self.rot * R.from_euler('z', 180, degrees=True)
            new_position = self.position

        self.position = new_position
        quat = self.rot.as_quat()

        timestamp = self.get_time_ms()
        msg = create_pose_message_from_arrays(quat, self.position, timestamp=timestamp)
        self.publisher_pose.publish(msg)
        self.get_logger().info(f"Pose has been published at time {timestamp}")


def main(args=None):
    rclpy.init(args=args)

    linear_pub = LinearPosePublisher()

    rclpy.spin(linear_pub)

    linear_pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
