import argparse
import csv
import os

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

import numpy as np

from audio_interfaces.msg import SignalsFreq, PoseRaw, CrazyflieStatus

# Time after which we are sure to have read the full bagfile. Set to something very high for no effect.
# Used to automate the process of bag file conversion (CTRL+C does not work for some reason)
TIMEOUT_S = np.inf # seconds

class CsvWriter(Node):
    def __init__(self):
        super().__init__("csv_writer")

        self.subscription_signals_f = self.create_subscription(
            SignalsFreq, "audio/signals_f", self.listener_callback_signals_f, 10
        )

        self.subscription_pose_raw = self.create_subscription(
            PoseRaw, "geometry/pose_raw", self.listener_callback_pose_raw, 10
        )

        self.subscription_status = self.create_subscription(
            CrazyflieStatus, "crazyflie/status", self.listener_callback_status, 10
        )

        self.declare_parameter("filename")
        self.set_parameters_callback(self.set_params)
        
        self.reset()
        self.get_logger().info('Subscribed to audio/signals_f and geometry/pose_raw.')

    def reset(self):
        self.header = {"index", "topic"}
        self.rows = []

    def listener_callback_signals_f(self, msg):
        row_dict = {
          "index": len(self.rows),
          "topic": "audio/signals_f",
          # TODO(FD): can we read these fields automatically?  
          "timestamp": msg.timestamp,
          "audio_timestamp": msg.audio_timestamp,
          "n_mics": msg.n_mics,
          "n_frequencies": msg.n_frequencies,
          "frequencies": np.array(msg.frequencies),
          "signals_real_vect": np.array(msg.signals_real_vect),
          "signals_imag_vect": np.array(msg.signals_imag_vect),
          "mic_positions": np.array(msg.mic_positions),
        }

        self.header = set(row_dict.keys()).union(self.header)
        self.rows.append(row_dict)

    def listener_callback_pose_raw(self, msg):
        row_dict = {
          "index": len(self.rows),
          "topic": "geometry/pose_raw",
          # TODO(FD): can we read these fields automatically?  
          "timestamp": msg.timestamp,
          "dx": msg.dx,
          "dy": msg.dy,
          "z": msg.z,
          "yaw_deg": msg.yaw_deg,
          "yaw_rate_deg": msg.yaw_rate_deg,
          "source_direction_deg": msg.source_direction_deg,
        }
        self.header = set(row_dict.keys()).union(self.header)
        self.rows.append(row_dict)

    def listener_callback_status(self, msg):
        row_dict = {
          "index": len(self.rows),
          "topic": "crazyflie/status",
          "timestamp": msg.timestamp,
          "vbat": msg.vbat,
        }
        self.header = set(row_dict.keys()).union(self.header)
        self.rows.append(row_dict)

    def set_params(self, params):
        """ Set the parameter. If filename is set, we save the current rows and reset. """
        param = params[0] # only one parameter possible.
        filename = param.get_parameter_value().string_value

        if filename == '':
            self.get_logger().info('resetting csv_writer')
            self.reset()
            return SetParametersResult(successful=True)

        if filename[-4:] != '.csv':
            filename = filename + '.csv'

        self.write_file(filename)
        return SetParametersResult(successful=True)

    def write_file(self, fullname):
        if not self.rows:
            self.get_logger().info("Empty rows, not saving.")
            return

        if not os.path.exists(fullname):
            with open(fullname, "w+") as f:
                csv_writer = csv.DictWriter(f, sorted(self.header))
                csv_writer.writeheader()
            self.get_logger().info(f"Wrote header in new file {fullname}.")
        else:
            self.get_logger().warn(f"File {fullname} exists! Appending new rows to it.")

        with open(fullname, "a") as f:
            csv_writer = csv.DictWriter(f, sorted(self.header))
            csv_writer.writerows(self.rows)

        self.get_logger().info(f"Appended {len(self.rows)} rows to {fullname}.")
        self.reset()


def main(args=None):
    import time 
    rclpy.init(args=args)

    writer = CsvWriter()
    try:
        start_time = time.time()
        while (time.time() - start_time) < TIMEOUT_S:
            rclpy.spin_once(writer)
        writer.get_logger().info("Timeout occured...")
    except KeyboardInterrupt:
        writer.get_logger().info("KeyboardInterrupt occured...")

    writer.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
