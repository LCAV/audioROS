"""
constant_pose_publisher.py: Constantly publish crazyflie's position
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from std_msgs.msg import String

class ConstantPosePublisher(Node):

    def __init__(self):
        super().__init__('constant_pose_publisher')
        
        self.publisher_pose = self.create_publisher(Pose, 'geometry/pose', 10)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Pose()
        
        msg.orientation.x=0.5
        msg.orientation.y=0.5
        msg.orientation.z=0.5
        msg.orientation.w=0.5
        
        msg.position.x=1.0
        msg.position.y=1.0
        msg.position.z=1.0
        
        self.publisher_pose.publish(msg)
        self.get_logger().info('Opublikowano pozycje')

def main(args=None):
    rclpy.init(args=args)

    const_pub = ConstantPosePublisher()

    rclpy.spin(const_pub)

    const_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()        

