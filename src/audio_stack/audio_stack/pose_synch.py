#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pose_synch.py: A class that does nothing but synchronize two topics, pose
and audio data. This way we are sure to get the best possible synchronization.
"""

import rclpy
from rclpy.node import Node

from audio_stack.topic_synchronizer import TopicSynchronizer
from audio_interfaces.msg import PoseRaw, SignalsFreq


class PoseSynch(Node):
    def __init__(self):
        super().__init__("pose_synch")

        self.pose_synch = TopicSynchronizer(20, self.get_logger(), n_buffer=10)
        self.subscription_pose = self.create_subscription(
            PoseRaw, "geometry/pose_raw", self.pose_synch.listener_callback, 10,
        )
        self.subscription_signals = self.create_subscription(
            SignalsFreq, "audio/signals_f", self.listener_callback, 10
        )
        self.publisher_pose_raw_synch = self.create_publisher(
            PoseRaw, "geometry/pose_raw_synch", 10
        )

    def listener_callback(self, msg_signals):
        # use the audio signal timestamp as reference for this measurement.
        timestamp = msg_signals.timestamp

        # Sometimes, this callback is called while the other TopicSynchronizer has not
        # registered the latest messages yet. therefore, we add a little buffer
        msg_pose = None
        i = 0
        timeout = 10
        while msg_pose is None:
            msg_pose = self.pose_synch.get_latest_message(timestamp, verbose=False)
            i += 1
            if i > timeout:
                self.get_logger().warn(
                    f"did not register valid pose at time {msg_signals.timestamp}"
                )
                return

        self.get_logger().info(
            f"for audio {timestamp}, using pose {msg_pose.timestamp}. lag: {msg_pose.timestamp - timestamp}ms"
        )
        self.publisher_pose_raw_synch.publish(msg_pose)


def main(args=None):
    rclpy.init(args=args)
    pose_synch = PoseSynch()
    rclpy.spin(pose_synch)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
