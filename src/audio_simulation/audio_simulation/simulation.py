"""
simulation.py:
"""

import rclpy
from rclpy.node import Node


from audio_interfaces.msg import Signals
from geometry_msgs.msg import Pose
from std_msgs.msg import String

N_MICS = 4

class AudioSimulation(Node):

    def __init__(self):
        super().__init__('audio_simulation')

        self.publisher_signals = self.create_publisher(String, 'OK_NOK', 10)
        self.subscription_position = self.create_subscription(Pose, 'crazyflie_position', self.listener_callback, 10)

    def listener_callback(self, msg_received):
        msg = String()
        msg.data = 'Something was sent'
        self.publisher_signals.publish(msg)
        self.get_logger().info('Received data')


def main(args=None):
    rclpy.init(args=args)

    new_pubsub = AudioSimulation()

    rclpy.spin(new_pubsub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    new_pubsub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()        


#geometry_msgs/Pose pose
#    geometry_msgs/Point position
#        float64 x
#        float64 y
#        float64 z
#    geometry_msgs/Quaternion orientation
#        float64 x
#        float64 y
#        float64 z
#        float64 w
