import logging
import os
import sys
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import PoseStamped

from audio_interfaces.msg import (
    CrazyflieMotors,
    CrazyflieStatus,
    GroundTruth,
    PoseRaw,
    SignalsFreq,
)
from audio_interfaces_py.messages import (
    create_pose_message,
    create_pose_raw_message,
    create_signals_freq_message,
    create_ground_truth_message,
)

from crazyflie_crtp.reader_crtp import ReaderCRTP
from crazyflie_description_py.parameters import (
    MIC_POSITIONS,
    N_MICS,
    FS,
    N_BUFFER,
)
from crazyflie_description_py.experiments import (
    WALL_DISTANCE_M,
    WALL_ANGLE_DEG,
    STARTING_POS,
    STARTING_YAW_DEG,
)

logging.basicConfig(level=logging.ERROR)
cf_id = "E7E7E7E7E7"
id = f"radio://0/80/2M/{cf_id}"

# cf_id = "E7E7E7E7E7"
# id = f"radio://0/70/2M/{cf_id}"

MAX_YLIM = 1e13  # set to inf for no effect.
MIN_YLIM = 1e-13  # set to -inf for no effect.


# parameter default values, will be overwritten by
# parameter yaml file, given by:
# ros2 run crazyflie_crtp gateway --ros-args --params-file params/default.yaml
PARAMS_DICT = {
    "send_audio_enable": (rclpy.Parameter.Type.INTEGER, 1),
    "min_freq": (rclpy.Parameter.Type.INTEGER, 1000),
    "max_freq": (rclpy.Parameter.Type.INTEGER, 6000),
    "delta_freq": (rclpy.Parameter.Type.INTEGER, 100),  # not used without prop
    "n_average": (rclpy.Parameter.Type.INTEGER, 5),  # not used without snr
    "bin_selection": (rclpy.Parameter.Type.INTEGER, 3),
    "filter_prop_enable": (rclpy.Parameter.Type.INTEGER, 0),
    "window_type": (rclpy.Parameter.Type.INTEGER, 1),
    "all": (rclpy.Parameter.Type.INTEGER, 0),
    "m1": (rclpy.Parameter.Type.INTEGER, 0),
    "m2": (rclpy.Parameter.Type.INTEGER, 0),
    "m3": (rclpy.Parameter.Type.INTEGER, 0),
    "m4": (rclpy.Parameter.Type.INTEGER, 0),
    "hover_height": (rclpy.Parameter.Type.DOUBLE, 0.0),
    "turn_angle": (rclpy.Parameter.Type.INTEGER, 0),
    "land_velocity": (rclpy.Parameter.Type.DOUBLE, 0.0),
    "move_distance": (rclpy.Parameter.Type.DOUBLE, 0.0),
    "buzzer_idx": (rclpy.Parameter.Type.INTEGER, 0),
    # "buzzer_effect": (rclpy.Parameter.Type.INTEGER, -1),
    # "buzzer_freq": (rclpy.Parameter.Type.INTEGER, 0),
}


class Gateway(Node):
    def __init__(self, reader_crtp):
        super().__init__(
            "gateway",
            automatically_declare_parameters_from_overrides=True,
            allow_undeclared_parameters=True,
        )

        self.start_time = time.time()

        self.publisher_signals = self.create_publisher(
            SignalsFreq, "audio/signals_f", 10
        )
        self.publisher_motion_pose = self.create_publisher(
            PoseStamped, "geometry/pose", 10
        )
        self.publisher_motion_pose_raw = self.create_publisher(
            PoseRaw, "geometry/pose_raw", 10
        )
        self.publisher_ground_truth = self.create_publisher(
            GroundTruth, "geometry/ground_truth", 10
        )
        self.publisher_status = self.create_publisher(
            CrazyflieStatus, "crazyflie/status", 10
        )
        self.publisher_motors = self.create_publisher(
            CrazyflieMotors, "crazyflie/motors", 10
        )

        # TODO(FD) should be an action service!
        self.subsription_commands = self.create_subscription(
            CrazyflieCommands, "crazyflie/commands", self.listener_callback_commands, 10
        )

        self.ground_truth_published = False
        self.starting_pose = None

        self.reader_crtp = reader_crtp

        self.set_parameters_callback(self.set_params)

        # need to do this to send initial parameters
        # over to Crazyflie.
        parameters = self.get_parameters(PARAMS_DICT.keys())
        self.set_parameters(parameters)

        # choose high publish rate so that we introduce as little
        # waiting time as possible
        desired_rate = 1000  # Hz
        self.create_timer(1 / desired_rate, self.publish_current_data)

    def listener_callback_commands(self, msg):
        self.get_logger().info(
            f"Command received: {msg.timestamp, msg.command_name, msg.command_value}."
        )
        found_command = self.send_command(msg.command_name, msg.command_value)
        if not found_command:
            self.get_logger().warn(f"Unknown command!")

    def publish_current_data(self):
        if not self.reader_crtp.audio_dict["published"]:
            self.publish_audio_dict()
            self.reader_crtp.audio_dict["published"] = True

        if not self.reader_crtp.logging_dicts["motion"]["published"]:
            self.publish_motion_dict()
            self.reader_crtp.logging_dicts["motion"]["published"] = True

        if not self.reader_crtp.logging_dicts["motors"]["published"]:
            self.publish_motors_dict()
            self.reader_crtp.logging_dicts["motors"]["published"] = True

        if not self.reader_crtp.logging_dicts["status"]["published"]:
            self.publish_battery()
            self.reader_crtp.logging_dicts["status"]["published"] = True

        if not self.ground_truth_published:
            self.publish_ground_truth()
            # self.ground_truth_published = True

    def publish_battery(self):
        msg = CrazyflieStatus()
        data = self.reader_crtp.logging_dicts["status"]["data"]
        if data is not None:
            msg.vbat = float(data["vbat"])
        else:
            msg.vbat = 0.0
        msg.timestamp = self.reader_crtp.logging_dicts["status"]["timestamp"]
        self.publisher_status.publish(msg)

    def publish_ground_truth(self):
        msg = create_ground_truth_message(
            wall_distance_m=WALL_DISTANCE_M, wall_angle_deg=WALL_ANGLE_DEG
        )
        self.publisher_ground_truth.publish(msg)

    def publish_motors_dict(self):
        msg = CrazyflieMotors()
        data = self.reader_crtp.logging_dicts["motors"]["data"]
        if data is not None:
            msg.motors_pwm = [float(data[f"m{i}_pwm"]) for i in range(1, 5)]
            msg.motors_thrust = [float(data[f"m{i}_thrust"]) for i in range(1, 5)]
        else:
            msg.motors_pwm = []
            msg.motors_thrust = []
        msg.timestamp = self.reader_crtp.logging_dicts["motors"]["timestamp"]
        self.publisher_motors.publish(msg)

    def publish_audio_dict(self):
        self.get_logger().info("publish audio dict")
        # read audio
        signals_f_vect = self.reader_crtp.audio_dict["signals_f_vect"]
        if signals_f_vect is None:
            self.get_logger().info("Empty audio. Not publishing")
            return

        # read frequencies
        fbins = self.reader_crtp.audio_dict["fbins"]
        if fbins is None:
            self.get_logger().info("No data yet. Not publishing")
            return

        all_frequencies = np.fft.rfftfreq(n=N_BUFFER, d=1 / FS)
        n_frequencies = len(fbins)

        # the only allowed duplicates are 0
        # if len(set(fbins[fbins>0])) < len(fbins[fbins>0]):
        # self.get_logger().warn(f"Duplicate values in fbins! unique values:{len(set(fbins))}")
        # return
        if not np.any(fbins > 0):
            self.get_logger().info(f"Empty fbins!")
            return
        elif np.any(fbins >= len(all_frequencies)):
            self.get_logger().warn(
                f"Invalid fbins! {fbins[fbins > len(all_frequencies)]}"
            )
            return

        frequencies = all_frequencies[fbins]

        signals_f = np.zeros((N_MICS, n_frequencies), dtype=np.complex128)
        for i in range(N_MICS):
            signals_f[i].real = signals_f_vect[i :: N_MICS * 2]
            signals_f[i].imag = signals_f_vect[i + N_MICS :: N_MICS * 2]

        abs_signals_f = np.abs(signals_f)
        if np.any(abs_signals_f > N_BUFFER):
            self.get_logger().warn("Possibly invalid audio:")
            xx, yy = np.where(abs_signals_f > N_BUFFER)
            self.get_logger().warn(f"at indices: {xx}, {yy}")
            self.get_logger().warn(f"values: {abs_signals_f[xx, yy]}")

        mic_positions_arr = np.array(MIC_POSITIONS)
        msg = create_signals_freq_message(
            signals_f.T,
            frequencies,
            mic_positions_arr,
            self.reader_crtp.audio_dict["timestamp"],
            self.reader_crtp.audio_dict["audio_timestamp"],
            FS,
        )
        self.publisher_signals.publish(msg)

        self.get_logger().warn(
            # f"{msg.timestamp}: Published audio data with fbins {fbins[fbins>0][[0, 1, 2, -1]]} and timestamp {msg.audio_timestamp}"
            f"{msg.timestamp}: Published audio data with fbins {fbins[fbins>0]} and timestamp {msg.audio_timestamp}"
        )

    def publish_motion_dict(self):
        from copy import deepcopy

        motion_dict = deepcopy(self.reader_crtp.logging_dicts["motion"]["data"])
        timestamp = self.reader_crtp.logging_dicts["motion"]["timestamp"]

        msg_pose_raw = create_pose_raw_message(**motion_dict, timestamp=timestamp)
        self.publisher_motion_pose_raw.publish(msg_pose_raw)

        # TODO(FD) generalize below to 3D
        # make sure we start at starting_pose, compensating for random offset
        # coming from the Crazyflie drone
        if self.starting_pose is None:
            self.starting_pose = [
                STARTING_POS[0] - motion_dict["x"],
                STARTING_POS[1] - motion_dict["y"],
                STARTING_POS[2] - motion_dict["z"],
                STARTING_YAW_DEG - motion_dict["yaw_deg"],
            ]
        msg_pose = create_pose_message(
            x=motion_dict["x"] + self.starting_pose[0],
            y=motion_dict["y"] + self.starting_pose[1],
            z=motion_dict["z"] + self.starting_pose[2],
            yaw_deg=motion_dict["yaw_deg"] + self.starting_pose[3],
            timestamp=timestamp,
        )
        self.publisher_motion_pose.publish(msg_pose)
        self.get_logger().debug(f"{msg_pose_raw.timestamp}: Published motion data.")

    def set_params(self, params):
        """ Overwrite the function set_params by NodeWithParams. 
        We need this so we commute new parameters directly to the Crazyflie drone.
        """
        success = True

        for param in params:

            # we need this in case this parameter
            # was not set yet at startup.
            # in that case we use the default values.
            if param.type_ == param.Type.NOT_SET:
                param = rclpy.parameter.Parameter(param.name, *PARAMS_DICT[param.name])

            if param.name == "send_audio_enable":
                self.get_logger().info(f"set {param.name} to {param.value}")
                # needs to also reset the arrays.
                self.reader_crtp.send_audio_enable(param.value)

            else:
                param_is_command = self.send_command(param.name, param.value)
                if not param_is_command:
                    self.set_audio_param(param)
        return SetParametersResult(successful=True)

    def send_command(self, param_name, param_value):
        if param_name == "hover_height":
            if param_value > 0:
                self.reader_crtp.send_hover_command(param_value)
            return True
        elif param_name == "turn_angle":
            if param_value != 0:
                self.reader_crtp.send_turn_command(param_value)
            return True
        elif param_name == "land_velocity":
            if param_value > 0:
                self.reader_crtp.send_land_command()
            return True
        elif param_name == "move_distance":
            # move by given distance, blocking
            if param_value != 0:
                self.reader_crtp.send_move_command(param_value)
            return True
        elif param_name == "move_forward":
            # move by given velocity, non-blocking
            if param_value != 0:
                self.get_logger().warn(f"send forward command {param_value}")
                self.reader_crtp.send_forward_command(param_value)
            return True
        elif param_name == "buzzer_idx":
            if param_value >= 0:
                self.get_logger().info(f"set {param_name} to {param_value}")
                self.reader_crtp.send_buzzer_idx(param_value)
            return True
        elif param_name == "all":
            self.get_logger().info(f"set all motors to {param_value}")
            self.reader_crtp.send_thrust_command(param_value)
            return True
        elif param_name in [f"m{i}" for i in range(1, 5)]:
            self.get_logger().info(f"set {param_name} to {param_value}")
            self.reader_crtp.send_thrust_command(param_value, param_name)
            return True
        return False

    def set_audio_param(self, param):
        old_value = self.get_parameter(param.name).value
        try:
            self.reader_crtp.cf.param.set_value(f"audio.{param.name}", param.value)
            self.get_logger().info(
                f"changed {param.name} from {old_value} to {param.value}"
            )
        except:
            self.get_logger().warn(f"error when trying to set {param.name}")


def main(args=None):
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    import cflib.crtp

    verbose = False
    log_motion = True  # get position logging from Crazyflie.
    log_motors = True

    cflib.crtp.init_drivers(enable_debug_driver=False)
    rclpy.init(args=args)

    with SyncCrazyflie(id) as scf:
        cf = scf.cf
        reader_crtp = ReaderCRTP(
            cf, verbose=verbose, log_motion=log_motion, log_motors=log_motors
        )
        node = Gateway(reader_crtp)

        try:
            rclpy.spin(node)
        except Exception as e:
            print("Node interrupted: Exception", e)
            reader_crtp.send_audio_enable(0)
            cf.param.set_value("motorPowerSet.enable", 0)
            reader_crtp.send_buzzer_freq(0)
            reader_crtp.send_buzzer_idx(0)
            print("Stop buzzer, motors, and audio sending, wait for 1s...")
            time.sleep(1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
