#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
wall_mapper.py: 
"""

import rclpy
from rclpy.action import ActionServer

from audio_interfaces.action import StateMachine
from audio_interfaces.msg import Distribution, PoseRaw
from audio_stack.topic_synchronizer import TopicSynchronizer
from audio_gtsam.wall_backend import WallBackend

from audio_interfaces_py.node_with_params import NodeWithParams

from crazyflie_demo.wall_detection import State


class WallMapper(NodeWithParams):
    """ Node to map out the walls of a room using audio signals  """

    PARAMS_DICT = {}

    def __init__(self):
        super().__init__("wall_mapper")

        self.subscription_dist_moving = self.create_subscription(
            Distribution, "results/distribution_moving", self.listener_callback_dist, 10
        )

        self.pose_synch = TopicSynchronizer(10, self.get_logger())
        self.subscription_pose = self.create_subscription(
            PoseRaw, "geometry/pose_raw", self.pose_synch.listener_callback, 10,
        )

        self._action_server = ActionServer(
            self, StateMachine, "state_machine", self.server_callback
        )

        # initialize ISAM
        self.wall_backend = WallBackend()

    def listener_callback_dist(self, msg):
        # extract the current probability distribution
        msg_pose = self.pose_synch.get_latest_message(msg.timestamp)
        if msg_pose is None:
            return

        r_world, v_world, yaw, yaw_rate = read_pose_raw_message(msg_pose)
        # add pose factor
        self.wall_backend.add_pose(r_world, yaw)

        # add plane factor
        distances = msg_dist.values
        probs = msg_dist.probabilities

        self.wall_backend.add_plane_from_distances(distances, probs)

    def server_callback(self, goal_handle):

        msg = goal_handle.request
        # find which enum the state corresponds to
        state_by_server = State(msg.state)
        self.get_logger().info(
            f"Action received: {msg.state} which corresponds to {self.state_by_server}"
        )
        result = StateMachine.Result()
        feedback = StateMachine.Feedback()
        if state_by_server == State.WAIT_DISTANCE:
            if self.wall_backend.check_wall():
                feedback.message = "Wall in moving direction!"
                result.flag = State.AVOID_DISTANCE
            else:
                feedback.message = "No wall detected."
                result.flag = State.WAIT_DISTANCE
            result.message = f"WallMapper: change state based on distance-check"
        else:
            result.flag = 0
            result.message = (
                f"WallMapper has nothing to do when state is {state_by_server}"
            )

        goal_handle.publish_feedback(feedback)
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    wall_mapper = WallMapper()
    rclpy.spin(wall_mapper)
    wall_mapper.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
