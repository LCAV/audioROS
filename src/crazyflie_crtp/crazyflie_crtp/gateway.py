import logging
import os
import sys
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
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
from audio_interfaces.srv import CrazyflieCommands
from audio_stack.parameters import WINDOW_CORRECTION

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

DEBUG = True  # some extra verbose stuff


class Gateway(Node):
    # parameter default values, will be overwritten by
    # parameter yaml file, given by:
    # ros2 run crazyflie_crtp gateway --ros-args --params-file params/default.yaml
    PARAMS_DICT = {
        "send_audio_enable": 1,
        "min_freq": 1000,
        "max_freq": 6000,
        "delta_freq": 100,  # not used without prop
        "n_average": 5,  # not used without snr
        "bin_selection": 5,
        "filter_prop_enable": 0,
        "window_type": 1,
        "all": 0,
        "m1": 0,
        "m2": 0,
        "m3": 0,
        "m4": 0,
        "hover_height": 0.0,
        "turn_angle": 0,
        "land_velocity": 0.0,
        "move_distance": 0.0,
        "buzzer_idx": 0,
    }

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

        self._server_command = self.create_service(
            CrazyflieCommands, "send_command_manual", self.commands_callback,
        )

        self.ground_truth_published = False
        self.starting_pose = None

        self.reader_crtp = reader_crtp

        self.add_on_set_parameters_callback(self.set_params)

        # fill parameters with default values, unless otherwise specified
        # by parameters file.
        params = self.get_parameters(self.PARAMS_DICT.keys())
        self.set_parameters(params)

        # choose high publish rate so that we introduce as little
        # waiting time as possible
        desired_rate = 1000  # Hz
        self.create_timer(1 / desired_rate, self.publish_current_data)

    def commands_callback(self, request, response):
        self.get_logger().info(
            f"Command received at time {request.timestamp}, {request.command_name}, {request.command_value:.2f}."
        )
        success = self.send_command(request.command_name, request.command_value)
        if not success:
            self.get_logger().info(f"Failed to send command")
            response.message = "Fail"
            response.value = 0.0
        else:
            response.message = "Success"
            response.value = 1.0
        return response

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

        window_type = self.get_parameter("window_type").value
        if window_type is not None:
            signals_f /= WINDOW_CORRECTION[window_type]
        else:
            self.get_logger().warn("window type is None")

        if DEBUG:
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

        self.get_logger().info(
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
        """ Instead of using set_params by NodeWithParams. 
        We need this so we commute new parameters directly to the Crazyflie drone.
        """
        for param in params:
            # we need this in case this parameter was not set yet at startup.
            # in that case we use the default values.
            # TODO: althought these default values are sent to
            # the Crazyflie as desired, the parameter is not actually
            # set with these values and will not be listed by the cli.
            if param.type_ == param.Type.NOT_SET:
                param = rclpy.parameter.Parameter(
                    param.name, value=self.PARAMS_DICT[param.name]
                )

            if param.name == "send_audio_enable":
                # needs to also reset the arrays.
                if self.reader_crtp is not None:
                    self.reader_crtp.send_audio_enable(param.value)
            else:
                param_is_command = self.send_command(param.name, param.value)
                if not param_is_command:
                    self.set_audio_param(param)
        return SetParametersResult(successful=True)

    def send_command(self, param_name, param_value):
        param_value = float(param_value)
        if param_name == "hover_height":
            if param_value > 0:
                success = self.reader_crtp.send_hover_command(param_value)
                if not success:
                    self.get_logger().warn(f"no battery, or not monitoring.")
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
                try:
                    self.reader_crtp.send_move_command(param_value)
                except Exception as e:
                    # self.get_logger().warn(f"Error in send_forward_command: {e}")
                    return False
            return True
        elif param_name == "move_forward":
            # move by given velocity, non-blocking
            if param_value != 0:
                # self.get_logger().warn(f"send forward command {param_value:.2f}")
                try:
                    self.reader_crtp.send_forward_command(param_value)
                except Exception as e:
                    # self.get_logger().warn(f"Error in send_forward_command: {e}")
                    return False
            return True
        elif param_name == "buzzer_idx":
            if param_value >= 0:
                self.get_logger().info(f"set {param_name} to {param_value}")
                self.reader_crtp.send_buzzer_idx(int(param_value))
            return True
        elif param_name == "all":
            self.get_logger().info(f"set all motors to {param_value}")
            self.reader_crtp.send_thrust_command(int(param_value))
            return True
        elif param_name in [f"m{i}" for i in range(1, 5)]:
            self.get_logger().info(f"set {param_name} to {param_value}")
            self.reader_crtp.send_thrust_command(int(param_value), param_name)
            return True
        return False

    def set_audio_param(self, param):
        old_value = self.get_parameter(param.name).value
        try:
            self.reader_crtp.set_audio_param(param)
            self.get_logger().info(
                f"changed audio parameter {param.name} from {old_value} to {param.value}"
            )
        except Exception as e:
            self.get_logger().warn(f"error when trying to set {param.name}: {e}")


def main(args=None):
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    import cflib.crtp

    verbose = False
    log_motion = True
    log_motors = True
    log_status = True

    cflib.crtp.init_drivers(enable_debug_driver=False)
    rclpy.init(args=args)

    with SyncCrazyflie(id) as scf:
        cf = scf.cf
        reader_crtp = ReaderCRTP(
            crazyflie=cf,
            verbose=verbose,
            log_motion=log_motion,
            log_motors=log_motors,
            log_status=log_status,
        )
        node = Gateway(reader_crtp)

        rclpy.spin(node)
        print("Stop buzzer, motors, and audio sending, wait for 1s...")
        reader_crtp.send_audio_enable(0)
        reader_crtp.send_disable_motors()
        reader_crtp.send_buzzer_idx(0)
        time.sleep(1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
