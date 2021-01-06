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

        self.fig, self.axs = plt.subplots(1, 2)
        self.axs[0].set_title("Motors pwm signals")
        self.axs[1].set_title("Motors thrust signals")
        self.axs[0].set_xlabel("time [s]")
        self.axs[1].set_xlabel("time [s]")
        self.axs[0].set_ylabel("pwm signals")
        self.axs[1].set_ylabel("thrust signals")
        #self.ax.set_ylim(3, 5)
        for i in range(4):
            self.axs[0].scatter([], [], color=f"C{i}", label=f"motor {i}")
            self.axs[1].scatter([], [], color=f"C{i}", label=f"motor {i}")
        self.axs[0].legend()
        self.axs[1].legend()
        self.axs[0].set_ylim(35000,66000)
        self.axs[1].set_ylim(35000,66000)
        self.fig.set_size_inches(14, 5)
        plt.show(block=False)

    def listener_callback_motors(self, msg):
        for i in range(4): 
            self.axs[0].scatter(msg.timestamp/1000, msg.motors_pwm[i], color=f"C{i}")
            self.axs[1].scatter(msg.timestamp/1000, msg.motors_thrust[i], color=f"C{i}")
        self.fig.canvas.draw()

def main(args=None):
    rclpy.init(args=args)

    plotter = MotorsPlotter()

    rclpy.spin(plotter)

    plotter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main
