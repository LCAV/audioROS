"""
constant_pose_publisher.py: Publish a constant pose of Crazyflie.
"""

import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

from audio_simulation.geometry import get_starting_pose_msg

class ConstantPosePublisher(Node):
    def __init__(self):
        super().__init__("constant_pose_publisher")

        self.publisher_pose = self.create_publisher(
            PoseStamped, "geometry/pose", 10
        )

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def get_time_ms(self):
        return int(round(time.time() * 1e-3))

    def timer_callback(self):
        msg = get_starting_pose_msg(
                timestamp_ms=self.get_time_ms()
        )
        self.publisher_pose.publish(msg)
        self.get_logger().info("Pose has been published")


def main(args=None):
    rclpy.init(args=args)

    const_pub = ConstantPosePublisher()

    rclpy.spin(const_pub)

    const_pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
