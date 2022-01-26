import csv
import os

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

import numpy as np


from audio_interfaces.msg import (
    SignalsFreq,
    PoseRaw,
    CrazyflieStatus,
    CrazyflieMotors,
)

# Time after which we are sure to have read the full bagfile. Set to something very high for no effect.
# Used to automate the process of bag file conversion (CTRL+C does not work for some reason)
TIMEOUT_S = np.inf  # seconds


class CsvHelper(object):
    def __init__(self):
        self.reset()

    def reset(self):
        self.header = {"index", "topic"}
        self.rows = []

    def callback_message(self, msg, topic):
        row_dict = {}
        for key, type_ in msg.get_fields_and_field_types().items():
            if "sequence" in type_:
                row_dict[key] = np.array(msg.__getattribute__(key))
            else:
                row_dict[key] = msg.__getattribute__(key)
        # row_dict = {key: msg.__getattribute__(key) for key in keys}
        row_dict["index"] = len(self.rows)
        row_dict["topic"] = topic

        self.header = set(row_dict.keys()).union(self.header)
        self.rows.append(row_dict)

    def write_file(self, fullname, logger=None):
        if not self.rows:
            if logger:
                logger.info("Empty rows, not saving.")
            return

        try:
            os.makedirs(os.path.dirname(fullname))
        except FileExistsError:
            pass

        with open(fullname, "w") as f:
            csv_writer = csv.DictWriter(f, sorted(self.header))
            csv_writer.writeheader()

            msg = f"Wrote header in new file {fullname}."
            if logger:
                logger.info(msg)
            csv_writer.writerows(self.rows)

            if logger:
                logger.info(f"Appended {len(self.rows)} rows to {fullname}.")
            self.reset()
        return True


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
            CrazyflieStatus, "crazyflie/status", self.listener_callback_status, 10,
        )

        self.subscription_motors = self.create_subscription(
            CrazyflieMotors, "crazyflie/motors", self.listener_callback_motors, 10,
        )

        self.declare_parameter("filename")
        self.add_on_set_parameters_callback(self.set_params)

        self.reset()
        self.get_logger().info(
            "Subscribed to audio/signals_f and crazyflie/status and crazyflie/motors and geometry/pose_raw."
        )
        self.csv_helper = CsvHelper()

    def reset(self):
        self.csv_helper.reset()

    def listener_callback_signals_f(self, msg):
        self.callback_message(msg, "audio/signals_f")

    def listener_callback_pose_raw(self, msg):
        self.callback_message(msg, "geometry/pose_raw")

    def listener_callback_status(self, msg):
        self.callback_message(msg, "crazyflie/status")

    def listener_callback_motors(self, msg):
        self.callback_message(msg, "crazyflie/motors")

    def callback_message(self, msg, topic):
        # fill the dictionary with all fields of this message.
        # if the field is an array, we convert it to a numpy array.
        self.csv_helper.callback_message(msg, topic)

    def set_params(self, params):
        """ Set the parameter. If filename is set, we save the current rows and reset. """
        param = params[0]  # only one parameter possible.
        filename = param.get_parameter_value().string_value

        if filename == "":
            self.get_logger().info("resetting csv_writer")
            self.reset()
            return SetParametersResult(successful=True)

        if filename[-4:] != ".csv":
            filename = filename + ".csv"

        self.csv_helper.write_file(filename)
        return SetParametersResult(successful=True)


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
