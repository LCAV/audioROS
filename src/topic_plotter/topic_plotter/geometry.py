#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
geometry.py: 
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

import numpy as np
from scipy.spatial.transform import Rotation

from audio_interfaces.msg import PoseRaw, DoaEstimates
from audio_interfaces_py.messages import read_pose_raw_message, read_pose_message
from audio_stack.topic_synchronizer import TopicSynchronizer
from audio_simulation.geometry import SOURCE_POS, ROOM_DIM, STARTING_POS

from .live_plotter import LivePlotter

MAX_LENGTH = 2 # number of positions to plot

def plot_room(ax):
    x, y = ROOM_DIM[:2]
    ax.plot([0, 0], [0, y], color='black')
    ax.plot([x, x], [0, y], color='black')
    ax.plot([0, x], [0, 0], color='black')
    ax.plot([0, x], [y, y], color='black')


def plot_source(ax):
    ax.scatter(SOURCE_POS[0], SOURCE_POS[1], color='black')


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
        self.pose_raw_list = np.array([STARTING_POS[:2]]).reshape((2, 1))

        # need no starting position for pose as it has absolute positions
        self.pose_list = np.empty((2, 0))

        # for error calculations
        self.error_list = []
        self.raw_pose_synch = TopicSynchronizer(20)
        self.subscription = self.create_subscription(PoseRaw, "geometry/pose_raw", self.raw_pose_synch.listener_callback, 10)

    def init_plotter(self, name, xlabel='x', ylabel='y'):
        if not (name in self.plotter_dict.keys()):
            self.plotter_dict[name] = LivePlotter(np.inf, -np.inf, label=name, log=False)
            self.plotter_dict[name].ax.set_xlabel(xlabel)
            self.plotter_dict[name].ax.set_ylabel(ylabel)
            self.plotter_dict[name].ax.axis('equal')

            plot_room(self.plotter_dict[name].ax)
            plot_source(self.plotter_dict[name].ax)

    def update_plotter(self, name, pose_list, yaw_deg, source_direction_deg=None):
        self.plotter_dict[name].update_scatter(
            pose_list[0, :], 
            pose_list[1, :]
        )

        self.plotter_dict[name].update_arrow(
                pose_list[:, -1], yaw_deg, name="yaw_deg"
        )

        if source_direction_deg is not None:
            self.plotter_dict[name].update_arrow(
                pose_list[:, -1], source_direction_deg, name="source_direction_deg"
            )

    def listener_callback_pose_raw(self, msg_pose_raw):
        xlabel = "x [m]"
        ylabel = "y [m]"
        self.init_plotter("pose raw", xlabel=xlabel, ylabel=ylabel)

        # TODO(FD) figure out what to do with yaw rate here
        d_world, yaw, yaw_rate = read_pose_raw_message(msg_pose_raw)
        new_position = self.pose_raw_list[:, -1] + d_world
        self.pose_raw_list = np.c_[self.pose_raw_list, new_position]
        if self.pose_raw_list.shape[1] > MAX_LENGTH:
            self.pose_raw_list = self.pose_raw_list[:, -MAX_LENGTH:]

        self.update_plotter("pose raw", self.pose_raw_list, yaw, msg_pose_raw.source_direction_deg)
        self.plotter_dict["pose raw"].fig.canvas.draw()

    def listener_callback_pose(self, msg_pose):
        xlabel = "x [m]"
        ylabel = "y [m]"
        self.init_plotter("pose", xlabel=xlabel, ylabel=ylabel)

        new_position, yaw, pitch, roll = read_pose_message(msg_pose)
        assert pitch == 0, pitch
        assert roll == 0, roll
        self.pose_list = np.c_[self.pose_list, new_position]

        if self.pose_list.shape[1] > MAX_LENGTH:
            self.pose_list = self.pose_list[:, -MAX_LENGTH:]

        self.update_plotter("pose", self.pose_list, yaw)
        self.plotter_dict["pose"].fig.canvas.draw()

    def listener_callback_doa(self, msg_doa):
        # plot the doa estimates in 2D plot
        xlabel = "x [m]"
        ylabel = "y [m]"
        self.init_plotter("pose raw", xlabel=xlabel, ylabel=ylabel)

        doa_estimates = list(msg_doa.doa_estimates_deg)

        for i, doa_estimate in enumerate(doa_estimates):
            self.plotter_dict["pose raw"].update_arrow(
                self.pose_raw_list[:, -1], doa_estimate, name=f"doa {i}"
            )
        self.plotter_dict["pose raw"].fig.canvas.draw()

        # calculate the current error 
        message = self.raw_pose_synch.get_latest_message(msg_doa.timestamp, self.get_logger())
        if message is not None:
            orientation = message.source_direction_deg
            error = abs(orientation - doa_estimates[0])
            self.error_list.append(error)
            avg_error = np.mean(self.error_list)
            self.get_logger().info(f"Current error: {error}, current average: {avg_error}")


def main(args=None):
    rclpy.init(args=args)

    plotter = GeometryPlotter()

    rclpy.spin(plotter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    plotter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
