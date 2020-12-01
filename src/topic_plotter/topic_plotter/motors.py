import rclpy
from rclpy.node import Node

import numpy as np
import matplotlib.pylab as plt

from audio_interfaces.msg import CrazyflieMotors

class MotorsPlotter(Node):
    def __init__(self):
        super().__init__("status_plotter")

        self.subscription_status = self.create_subscription(
            CrazyflieMotors, "crazyflie/motors", self.listener_callback_motors, 10
        )

        self.fig, self.ax = plt.subplots()
        self.ax.set_title("Motors pwm signals")
        self.ax.set_xlabel("time [s]")
        self.ax.set_ylabel("pwm signals")
        #self.ax.set_ylim(3, 5)
        for i in range(4):
            self.ax.scatter([], [], color=f"C{i}", label=f"motor {i}")
        self.ax.legend()
        self.fig.set_size_inches(14, 20)
        plt.show(block=False)

    def listener_callback_motors(self, msg):
        for i in range(4): 
            self.ax.scatter(msg.timestamp/1000, msg.motors_pwm[i], color=f"C{i}")
        self.fig.canvas.draw()

def main(args=None):
    rclpy.init(args=args)

    plotter = MotorsPlotter()

    rclpy.spin(plotter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    plotter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
