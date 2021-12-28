#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
wall_mapper.py: 
"""

import gtsam

X = gtsam.symbol_shorthand.X 
P = gtsam.symbol_shorthand.P 

class WallMapper(NodeWithParams):
    """ Node to map out the walls of a room using audio signals  """
    PARAMS_DICT = {}
    
    def __init__(self):
        super().__init__("wall_mapper")

        self.subscription_dist_moving = self.create_subscription(
            Signals, "results/distribution_moving", self.listener_callback_dist, 10
        )

        self.pose_synch = TopicSynchronizer(10, self.get_logger())
        self.subscription_pose = self.create_subscription(
            PoseRaw, "geometry/pose_raw", self.pose_synch.listener_callback, 10,
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
        distance = distances[np.argmax(probs)]
        # TODO(FD): remove ground truth angle estimate
        self.wall_backend.add_plane(distance, azimuth=np.pi/2, elevation=0)

        planes, poses = self.wall_backend.get_results()

def main(args=None):
    rclpy.init(args=args)
    wall_mapper = WallMapper()
    rclpy.spin(wall_mapper)
    wall_mapper.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
