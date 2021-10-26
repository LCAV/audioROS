import rclpy
from audio_interfaces import NodeWithParams


class WallDetection(NodeWithParams):
    PARAMS_DICT = {
        "mode": "user-input"  #
        # "mode": "automatic"
    }

    def __init__(self):
        super().__init__("wall_detection")

        # TODO(FD): should be an action client!
        self.publisher_commands = self.create_publisher(
            Commands, "crazyflie/commands", 10
        )

    # detect wall using continuous sweeps
    def do_fslices(self):
        pass

    # detect wall using mono frequency
    def do_dslice(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = WallDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

    publisher.destroy_node()
    rclpy.shutdown()
