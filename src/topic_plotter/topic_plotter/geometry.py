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
from .live_plotter import LivePlotter
from audio_stack.topic_synchronizer import TopicSynchronizer

MAX_LENGTH = 10 # number of positions to plot

class GeometryPlotter(Node):
    def __init__(self):
        super().__init__("geometry_plotter")

        self.subscription_pose_raw = self.create_subscription(
            PoseRaw, "geometry/pose_raw", self.listener_callback_pose_raw, 10
        )

        #self.subscription_pose = self.create_subscription(
        #    Pose, "geometry/pose", self.listener_callback_pose, 10
        #)

        self.subscription_doa = self.create_subscription(
            DoaEstimates, "geometry/doa_estimates", self.listener_callback_doa, 10
        )

        self.plotter_dict = {}
        # initialize a starting position for pose_raw, as it only contains delta positions
        self.pose_raw_list = np.zeros((2, 1))

        # need no starting position for pose as it has absolute positions
        self.pose_list = np.empty((2, 0))

        # for error calculations
        self.error_list = []
        self.raw_pose_synch = TopicSynchronizer(10)
        self.subscription = self.create_subscription(PoseRaw, "geometry/pose_raw", self.raw_pose_synch.listener_callback, 10)

    def init_plotter(self, name, xlabel='x', ylabel='y'):
        if not (name in self.plotter_dict.keys()):
            self.plotter_dict[name] = LivePlotter(np.inf, -np.inf, label=name, log=False)
            self.plotter_dict[name].ax.set_xlabel(xlabel)
            self.plotter_dict[name].ax.set_ylabel(ylabel)
            self.plotter_dict[name].ax.axis('equal')

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

        d_local = np.array((msg_pose_raw.dx, msg_pose_raw.dy))
        yaw = msg_pose_raw.yaw_deg
        r = Rotation.from_euler('z', yaw, degrees=True)
        d_world = r.as_matrix()[:2, :2] @ d_local
        previous_position = self.pose_raw_list[:, -1]
        self.pose_raw_list = np.c_[self.pose_raw_list, previous_position + d_world]

        if self.pose_raw_list.shape[1] > MAX_LENGTH:
            self.pose_raw_list = self.pose_raw_list[:, -MAX_LENGTH:]

        self.update_plotter("pose raw", self.pose_raw_list, yaw, msg_pose_raw.source_direction_deg)

    # TODO(FD) figure out why the pose_raw topic and pose_raw topic do not yield exactly the same 
    # position estimates.
    def listener_callback_pose(self, msg_pose):
        xlabel = "x [m]"
        ylabel = "y [m]"
        self.init_plotter("pose", xlabel=xlabel, ylabel=ylabel)

        new_position = np.array((msg_pose.position.x, msg_pose.position.y))
        self.pose_list = np.c_[self.pose_list, new_position]

        quat = [msg_pose.orientation.x, msg_pose.orientation.y, msg_pose.orientation.z, msg_pose.orientation.w]
        r = Rotation.from_quat(quat)
        [yaw, pitch, roll] = r.as_euler('zyx', degrees=True)
        assert pitch == 0, pitch
        assert roll == 0, roll
        if self.pose_list.shape[1] > MAX_LENGTH:
            self.pose_list = self.pose_list[:, -MAX_LENGTH:]

        self.update_plotter("pose", self.pose_list, yaw)

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
