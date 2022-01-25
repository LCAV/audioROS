from functools import partial
import time

import rclpy
from rclpy.node import Node

import matplotlib.pylab as plt

from audio_interfaces.msg import SignalsFreq, Distribution, PoseRaw

MESSAGES = {
    "audio/signals_f": SignalsFreq,
    "geometry/pose_raw_synch": PoseRaw,
    "results/distribution_moving": Distribution,
    "results/distribution_raw": Distribution,
}

N_COLORS = 10
REFERENCE = "audio/signals_f"
MAX_TIME_S = 10


class MessagePlotter(Node):
    def __init__(self):
        super().__init__("message_plotter")
        self.colors = {}

        self.fig, self.ax = plt.subplots()
        self.fig.set_size_inches(10, 5)
        self.fig.canvas.manager.set_window_title("Message timings")

        self.ax.set_title("Message timings")
        self.ax.set_xlabel("time [s]")
        self.ax.set_ylabel("topic")
        self.ax.set_ylim(-0.5, len(MESSAGES) - 0.5)

        # TODO(FD) for an unknown reason, using lambda for the callbacks below
        # doesn't work, the passed topic will always be the last one from MESSAGES.
        self.all_subscriptions = {}
        for topic, msg_type in MESSAGES.items():
            self.all_subscriptions[topic] = self.create_subscription(
                msg_type, topic, partial(self.listener_callback, topic), 10
            )

        self.ax.set_yticks(range(len(MESSAGES)))
        self.ax.set_yticklabels(MESSAGES.keys())

        # self.fig.set_size_inches(14, 10)
        self.start_time = None
        # self.fig.subplots_adjust(left=0.5)
        plt.tight_layout()
        plt.show(block=False)

    def get_time(self):
        t = time.time()
        if self.start_time is None:
            self.start_time = t
        return t - self.start_time

    def listener_callback(self, topic, msg):
        time_s = self.get_time()
        self.get_logger().info(
            f"Callback for topic {topic} at time {time_s:.1f}s since startup"
        )

        color = self.colors.get(msg.timestamp, None)
        if color is None:
            color_idx = len(self.colors) % N_COLORS
            color = f"C{color_idx}"
            self.colors[msg.timestamp] = color

            label = time_s
            self.get_logger().info(f"Added new timestamp {msg.timestamp}")
        else:
            label = None

        y = list(MESSAGES.keys()).index(topic)
        self.ax.scatter(time_s, y, color=color, label=label)

        # only show latest 3 minutes
        min_time_s = max(time_s - MAX_TIME_S, 0)
        self.ax.set_xlim(min_time_s, time_s)
        self.fig.canvas.draw()


def main(args=None):
    rclpy.init(args=args)

    plotter = MessagePlotter()

    rclpy.spin(plotter)

    plotter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
