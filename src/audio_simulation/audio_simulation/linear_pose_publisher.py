"""
linear_pose_publisher.py: Publish linear movement of Crazyflie inside a room. 
"""

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Pose
from std_msgs.msg import String

from audio_simulation.geometry import get_starting_pose
from audio_simulation.geometry import ROOM_DIM, STARTING_POS, YAW_DEG

VELOCITY = np.array([0., 0.5, 0.]) # m/s, in drone coordinates
EPS = 0.2 # m, safety margin from walls

class LinearPosePublisher(Node):
    def __init__(self):
        super().__init__("linear_pose_publisher")

        self.publisher_pose = self.create_publisher(Pose, "geometry/pose", 10)

        self.rot = R.from_euler('z', YAW_DEG, degrees=True)
        self.velocity = VELOCITY
        self.position = STARTING_POS
        assert np.all(self.position <= ROOM_DIM), "starting position not inside room!"

        # start movement
        self.timer_period = 0.5  # seconds
        # we do not need to wait before starting the timer
        # because the timer first waits for self.timer_period.
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        msg = get_starting_pose()
        self.publisher_pose.publish(msg)


    def timer_callback(self):
        delta = self.rot.apply(self.timer_period * self.velocity)
        new_position = self.position + delta

        # if we het the wall, revert direction
        if np.any(new_position > np.array(ROOM_DIM) - EPS) or np.any(new_position < EPS):
            #self.velocity = -self.velocity
            self.get_logger().warn("touched wall, turn direction")
            self.rot = self.rot * R.from_euler('z', 180, degrees=True)
            return

        self.position = new_position

        quat = self.rot.as_quat()

        msg = Pose()
        msg.orientation.x = quat[0]
        msg.orientation.y = quat[1]
        msg.orientation.z = quat[2]
        msg.orientation.w = quat[3]
        msg.position.x = self.position[0]
        msg.position.y = self.position[1]
        msg.position.z = self.position[2]

        self.publisher_pose.publish(msg)
        self.get_logger().info("Pose has been published")


def main(args=None):
    rclpy.init(args=args)

    linear_pub = LinearPosePublisher()

    rclpy.spin(linear_pub)

    linear_pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
