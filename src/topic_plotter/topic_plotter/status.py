import rclpy
from rclpy.node import Node

import matplotlib.pylab as plt

from audio_interfaces.msg import CrazyflieStatus

import sys
sys.path.append('crazyflie-audio/python')
from reader_crtp import ReaderCRTP

class StatusPlotter(Node):
    def __init__(self):
        super().__init__("status_plotter")

        self.subscription_status = self.create_subscription(
            CrazyflieStatus, "crazyflie/status", self.listener_callback_status, 10
        )

        self.fig, self.ax = plt.subplots()
        self.ax.set_title("Status")
        self.ax.set_xlabel("time [s]")
        self.ax.set_ylabel("battery level [V]")
        self.ax.set_ylim(2.5, 4.2)
        self.hline = self.ax.axhline(ReaderCRTP.BATTERY_OK, label='OK level', color="C0")
        self.hline_initial = None 
        self.ax.scatter([], [], color="C2", label='current level')
        self.ax.legend()
        #self.fig.set_size_inches(14, 10)
        self.fig.set_size_inches(5, 5)
        self.start_time = None
        plt.show(block=False)

    def listener_callback_status(self, msg):
        if self.hline_initial is None:
            self.hline_initial = self.ax.axhline(msg.vbat, label='initial level', color="C1")
            self.ax.legend()

        time_s = msg.timestamp/1000
        if self.start_time is None:
            self.start_time = time_s
        self.ax.scatter(time_s, msg.vbat, color="C2")
        self.ax.set_title(f"Status, current battery level: {msg.vbat:.2f}V")
        # only show latest 3 minutes
        min_time_s = max(time_s - 3 * 60, self.start_time)
        self.ax.set_xlim(min_time_s, time_s)
        self.fig.canvas.draw()


def main(args=None):
    rclpy.init(args=args)

    plotter = StatusPlotter()

    rclpy.spin(plotter)

    plotter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
