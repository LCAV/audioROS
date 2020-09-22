import csv
import os

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

import numpy as np

from audio_interfaces.msg import SignalsFreq, PoseRaw


class CsvWriter(Node):
    def __init__(self, filename="test.csv", dirname="experiments/"):
        super().__init__("csv_writer")

        self.subscription_signals_f = self.create_subscription(
            SignalsFreq, "audio/signals_f", self.listener_callback_signals_f, 10
        )

        self.subscription_pose_raw = self.create_subscription(
            PoseRaw, "geometry/pose_raw", self.listener_callback_pose_raw, 10
        )

        self.declare_parameter("filename")
        self.declare_parameter("dirname")
        parameter1 = rclpy.parameter.Parameter("filename", rclpy.Parameter.Type.STRING, filename)
        parameter2 = rclpy.parameter.Parameter("dirname", rclpy.Parameter.Type.STRING, dirname)
        self.set_parameters_callback(self.set_params)
        self.set_parameters([parameter1, parameter2])
        
        self.header = {"index", "topic"}
        self.rows = []
        self.index = 0

    def listener_callback_signals_f(self, msg):
        row_dict = {
          "index": self.index,
          "topic": "audio/signals_f",
          # TODO(FD): can we read these fields automatically?  
          "timestamp": msg.timestamp,
          "n_mics": msg.n_mics,
          "n_frequencies": msg.n_frequencies,
          "frequencies": np.array(msg.frequencies),
          "signals_real_vect": np.array(msg.signals_real_vect),
          "signals_imag_vect": np.array(msg.signals_imag_vect),
        }

        self.header = set(row_dict.keys()).union(self.header)
        self.rows.append(row_dict)
        self.index += 1

    def listener_callback_pose_raw(self, msg):
        row_dict = {
          "index": self.index,
          "topic": "geometry/pose_raw",
          # TODO(FD): can we read these fields automatically?  
          "timestamp": msg.timestamp,
          "dx": msg.dx,
          "dy": msg.dy,
          "z": msg.z,
          "yaw_deg": msg.yaw_deg,
          "source_direction-deg": msg.source_direction_deg,
        }
        self.header = set(row_dict.keys()).union(self.header)
        self.rows.append(row_dict)
        self.index += 1

    def set_params(self, params):
        dirname = self.get_parameter("dirname").get_parameter_value().string_value
        filename = self.get_parameter("filename").get_parameter_value().string_value
        for param in params:
            if param.name == "dirname":
                dirname = param.get_parameter_value().string_value
            elif param.name == "filename":
                filename = param.get_parameter_value().string_value

        if filename[-4:] != '.csv':
            filename = filename + '.csv'

        self.fullname = os.path.join(dirname, filename)
        print(f"will write {self.fullname} on KeyboardInterrupt.") 
        return SetParametersResult(successful=True)

    def write_file(self):
        if not self.rows:
            print("empty rows, not saving.")
            return

        with open(self.fullname, "a+") as f:
            csv_writer = csv.DictWriter(f, self.header)
            csv_writer.writeheader()
            csv_writer.writerows(self.rows)


def main(args=None):
    rclpy.init(args=args)

    writer = CsvWriter()

    try:
        rclpy.spin(writer)
    except KeyboardInterrupt:
        writer.write_file()

    writer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
