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

from audio_interfaces.msg import PoseRaw
from .live_plotter import LivePlotter

MAX_LENGTH = 10 # number of positions to plot

class GeometryPlotter(Node):
    def __init__(self):
        super().__init__("geometry_plotter")

        self.subscription_pose_raw = self.create_subscription(
            PoseRaw, "geometry/pose_raw", self.listener_callback_pose_raw, 10
        )

        self.subscription_pose = self.create_subscription(
            Pose, "geometry/pose", self.listener_callback_pose, 10
        )

        self.plotter_dict = {}
        self.pose_raw_list = np.zeros((2, 1))
        self.pose_list = np.empty((2, 0))


    def init_plotter(self, name, xlabel='x', ylabel='y'):
        if not (name in self.plotter_dict.keys()):
            self.plotter_dict[name] = LivePlotter(np.inf, -np.inf, label=name, log=False)
            self.plotter_dict[name].ax.set_xlabel(xlabel)
            self.plotter_dict[name].ax.set_ylabel(ylabel)
            self.plotter_dict[name].ax.axis('equal')

    def update_plotter(self, name, pose_list, yaw_deg):
        self.plotter_dict[name].update_scatter(
            pose_list[0, :], 
            pose_list[1, :]
        )

        arrow_length = max(np.max(pose_list) - np.min(pose_list), 1.0) / 0.3
        dx = arrow_length * np.cos(yaw_deg * np.pi / 180)
        dy = arrow_length * np.sin(yaw_deg * np.pi / 180)
        self.plotter_dict[name].update_arrow(
            pose_list[0, -1], 
            pose_list[1, -1],
            dx=dx, dy=dy
        )


    # TODO(FD) figure out why the pose_raw topic and pose_raw topic do not yield exactly the same 
    # position estimates.
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
        self.update_plotter("pose raw", self.pose_raw_list, yaw)



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
