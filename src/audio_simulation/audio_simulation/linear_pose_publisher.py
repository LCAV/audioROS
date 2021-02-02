"""
linear_pose_publisher.py: Publish linear movement of Crazyflie inside a room. 
"""

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Pose

from audio_simulation.geometry import get_starting_pose
from audio_simulation.geometry import ROOM_DIM, STARTING_POS, YAW_DEG

VELOCITY = np.array([0., 0.5, 0.]) # m/s, in drone coordinates
EPS = 0.2 # m, safety margin from walls
MAX_Y = None

class LinearPosePublisher(Node):
    def __init__(self, position=STARTING_POS, velocity=VELOCITY, yaw_deg=YAW_DEG, max_y=MAX_Y):
        super().__init__("linear_pose_publisher")

        self.publisher_pose = self.create_publisher(Pose, "geometry/pose", 10)

        self.rot = R.from_euler('z', yaw_deg, degrees=True)
        self.velocity = velocity
        self.position = position 
        self.max_y = max_y
        assert np.all(self.position <= ROOM_DIM-EPS) and np.all(self.position >= EPS), "starting position not inside room!"

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
            self.get_logger().warn("touched wall, turn direction")
            self.velocity = -self.velocity
            #self.rot = self.rot * R.from_euler('z', 180, degrees=True)
            return

        # if we reached max_y, revert direction. 
        if (self.max_y is not None) and (new_position[1] > self.max_y):
            self.get_logger().warn(f"reached {self.max_y}, turn direction")
            self.velocity = -self.velocity
            #self.rot = self.rot * R.from_euler('z', 180, degrees=True)
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
