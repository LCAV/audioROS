#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
geometry.py: 
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

import numpy as np

from audio_interfaces.msg import PoseRaw, DoaEstimates
from audio_interfaces_py.messages import read_pose_raw_message, read_pose_message
from audio_stack.topic_synchronizer import TopicSynchronizer
from audio_simulation.geometry import SPEAKER_POSITION, ROOM_DIM, STARTING_POS

from .live_plotter import LivePlotter

MAX_LENGTH = 1000  # number of positions to plot

XLABEL = "x [m]"
YLABEL = "y [m]"


def plot_room(ax):
    x, y = ROOM_DIM[:2]
    ax.plot([0, 0], [0, y], color="black")
    ax.plot([x, x], [0, y], color="black")
    ax.plot([0, x], [0, 0], color="black")
    ax.plot([0, x], [y, y], color="black")


def plot_source(ax):
    ax.scatter(SPEAKER_POSITION[0], SPEAKER_POSITION[1], color="black")


class GeometryPlotter(Node):
    def __init__(self):
        super().__init__("geometry_plotter")

        self.subscription_pose_raw = self.create_subscription(
            PoseRaw, "geometry/pose_raw", self.listener_callback_pose_raw, 10
        )

        self.subscription_pose = self.create_subscription(
            Pose, "geometry/pose", self.listener_callback_pose, 10
        )

        self.subscription_doa = self.create_subscription(
            DoaEstimates, "geometry/doa_estimates", self.listener_callback_doa, 10
        )

        self.plotter_dict = {}
        # initialize a starting position for pose_raw, as it only contains delta positions
        self.pose_motion_list = np.array([STARTING_POS[:2]]).reshape((2, 1))
        self.pose_imu_list = np.array([STARTING_POS[:2]]).reshape((2, 1))
        self.pose_kalman_list = np.array([STARTING_POS[:2]]).reshape((2, 1))
        self.kalman_start_pos = None
        self.previous_time = None

        # need no starting position for pose as it has absolute positions
        self.pose_list = np.empty((2, 0))

        # for error calculations
        self.error_list = []
        self.raw_pose_synch = TopicSynchronizer(20)
        self.subscription = self.create_subscription(
            PoseRaw, "geometry/pose_raw", self.raw_pose_synch.listener_callback, 10
        )

    def init_plotter(self, name, xlabel="x", ylabel="y"):
        if not (name in self.plotter_dict.keys()):
            self.plotter_dict[name] = LivePlotter(
                np.inf, -np.inf, label=name, log=False
            )
            self.plotter_dict[name].ax.set_xlabel(xlabel)
            self.plotter_dict[name].ax.set_ylabel(ylabel)
            self.plotter_dict[name].ax.axis("equal")

            # plot_room(self.plotter_dict[name].ax)
            # plot_source(self.plotter_dict[name].ax)

    def update_plotter(
        self,
        name,
        pose_list,
        yaw_deg=None,
        source_direction_deg=None,
        pose_label="pose estimates",
    ):
        self.plotter_dict[name].update_scatter(
            pose_list[0, :], pose_list[1, :], label=pose_label
        )

        if yaw_deg is not None:
            self.plotter_dict[name].update_arrow(
                pose_list[:, -1], yaw_deg, label="yaw_deg"
            )

        if source_direction_deg is not None:
            self.plotter_dict[name].update_arrow(
                pose_list[:, -1], source_direction_deg, label="source_direction_deg"
            )

        self.plotter_dict[name].ax.legend(loc="upper right")

    def listener_callback_pose_raw(self, msg_pose_raw):
        self.init_plotter("pose raw", xlabel=XLABEL, ylabel=YLABEL)

        r_world, d_motion, v_world, yaw, yaw_rate = read_pose_raw_message(msg_pose_raw)

        if self.previous_time is None:
            self.previous_time = msg_pose_raw.timestamp

        delta_sec = (msg_pose_raw.timestamp - self.previous_time) / 1000.0
        d_imu = v_world * delta_sec
        self.previous_time = msg_pose_raw.timestamp

        self.get_logger().info(
            f"yaw [deg]: {yaw:3.0f}, global dx [mm]: {d_motion[0]*1000:4.0f}, global dy [mm]:{d_motion[1]*1000:4.0f}"
        )

        pose_motion = self.pose_motion_list[:, -1] + d_motion
        self.pose_motion_list = np.c_[self.pose_motion_list, pose_motion]

        pose_imu = self.pose_imu_list[:, -1] + d_imu
        self.pose_imu_list = np.c_[self.pose_imu_list, pose_imu]

        # make sure we always start plotting at STARTING_POS, even though
        # the Kalman estiamte might be non-zero in the beginning.
        if self.kalman_start_pos is None:
            self.kalman_start_pos = r_world[:2] - STARTING_POS[:2]

        pose_kalman = r_world[:2] - self.kalman_start_pos
        self.pose_kalman_list = np.c_[self.pose_kalman_list, pose_kalman]

        if self.pose_motion_list.shape[1] > MAX_LENGTH:
            self.pose_motion_list = self.pose_motion_list[:, -MAX_LENGTH:]
        if self.pose_imu_list.shape[1] > MAX_LENGTH:
            self.pose_imu_list = self.pose_imu_list[:, -MAX_LENGTH:]
        if self.pose_kalman_list.shape[1] > MAX_LENGTH:
            self.pose_kalman_list = self.pose_kalman_list[:, -MAX_LENGTH:]

        # self.update_plotter("pose raw", self.pose_motion_list, yaw, msg_pose_raw.source_direction_deg, pose_label="motion")
        # self.update_plotter("pose raw", self.pose_imu_list, pose_label="imu")
        self.update_plotter("pose raw", self.pose_kalman_list, pose_label="kalman")
        # self.plotter_dict["pose raw"].ax.set_xlim(4.9, 5.3)
        # self.plotter_dict["pose raw"].ax.set_ylim(0.1, 0.5)
        self.plotter_dict["pose raw"].fig.canvas.draw()

    def listener_callback_pose(self, msg_pose):
        self.init_plotter("pose", xlabel=XLABEL, ylabel=YLABEL)

        new_position, yaw, pitch, roll = read_pose_message(msg_pose)
        assert pitch == 0, pitch
        assert roll == 0, roll
        self.pose_list = np.c_[self.pose_list, new_position]

        if self.pose_list.shape[1] > MAX_LENGTH:
            self.pose_list = self.pose_list[:, -MAX_LENGTH:]

        self.update_plotter("pose", self.pose_list, yaw)
        self.plotter_dict["pose"].fig.canvas.draw()

    def listener_callback_doa(self, msg_doa):
        self.init_plotter("pose raw", xlabel=XLABEL, ylabel=YLABEL)

        doa_estimates = list(msg_doa.doa_estimates_deg)

        for i, doa_estimate in enumerate(doa_estimates):
            self.plotter_dict["pose raw"].update_arrow(
                self.pose_motion_list[:, -1], doa_estimate, label=f"doa {i}"
            )
        self.plotter_dict["pose raw"].fig.canvas.draw()

        # calculate the current error
        message = self.raw_pose_synch.get_latest_message(
            msg_doa.timestamp, self.get_logger()
        )
        if message is not None:
            orientation = message.source_direction_deg
            error = abs(orientation - doa_estimates[0])
            self.error_list.append(error)
            avg_error = np.mean(self.error_list)
            self.get_logger().info(
                f"Current error: {error}, current average: {avg_error}"
            )


def main(args=None):
    rclpy.init(args=args)

    plotter = GeometryPlotter()

    rclpy.spin(plotter)

    plotter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
