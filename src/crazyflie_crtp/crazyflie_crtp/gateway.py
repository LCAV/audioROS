import logging
import os
import sys
import time

import numpy as np

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
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
from audio_interfaces_py.node_with_params import NodeWithParams

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
cf_id = "E7E7E7E7E8"
id = f"radio://0/80/2M/{cf_id}"

# cf_id = "E7E7E7E7E7"
# id = f"radio://0/70/2M/{cf_id}"

MAX_YLIM = 1e13  # set to inf for no effect.
MIN_YLIM = 1e-13  # set to -inf for no effect.


class Gateway(NodeWithParams):
    PARAMS_DICT = {
        "send_audio_enable": 1,
        "min_freq":  4000,
        "max_freq":  4200,
        "delta_freq":  100,  # not used without prop
        "n_average":  5,  # not used without snr
        "filter_snr_enable":  0,
        "filter_prop_enable":  0,
        "window_type":  1,
        "all":  0,
        "m1":  0,
        "m2":  0,
        "m3":  0,
        "m4":  0,
        "hover_height":  0.0,
        "turn_angle":  0,
        "land_velocity":  0.0,
        "move_distance":  0.0,
        "buzzer_idx":  0,
        # "buzzer_effect": (rclpy.Parameter.Type.INTEGER, -1),
        # "buzzer_freq": (rclpy.Parameter.Type.INTEGER, 0),
    }

    def __init__(self, reader_crtp):
        self.ground_truth_published = False
        self.starting_pose = None
        self.reader_crtp = reader_crtp
        self.start_time = time.time()

        super().__init__("gateway")
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

        # choose high publish rate so that we introduce as little
        # waiting time as possible
        self.desired_rate = 1000  # Hz
        self.create_timer(1 / self.desired_rate, self.publish_current_data)

    def publish_current_data(self):
        # only the first one is for my publisher
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
        # reader_crtp is the bluetooth reader for the crazyflie
        # read audio
        signals_f_vect = self.reader_crtp.audio_dict["signals_f_vect"]
        if signals_f_vect is None:
            self.get_logger().warn("Empty audio. Not publishing")
            return

        # read frequencies
        fbins = self.reader_crtp.audio_dict["fbins"]
        if fbins is None:
            self.get_logger().warn("No data yet. Not publishing")
            return

        all_frequencies = np.fft.rfftfreq(n=N_BUFFER, d=1 / FS)
        n_frequencies = len(fbins)

        # the only allowed duplicates are 0
        # if len(set(fbins[fbins>0])) < len(fbins[fbins>0]):
        # self.get_logger().warn(f"Duplicate values in fbins! unique values:{len(set(fbins))}")
        # return
        if not np.any(fbins > 0):
            self.get_logger().warn(f"Empty fbins!")
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

        self.get_logger().info(
            f"{msg.timestamp}: Published audio data with fbins {fbins[[0, 1, 2, -1]]} and timestamp {msg.audio_timestamp}"
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
        current_params = {param.name:param.value for param in params}
        for param_name, param_value in current_params.items():
            if param_value is None:
                continue
            # send motor commands
            if param_name == "hover_height":
                if param_value > 0:
                    success = self.reader_crtp.send_hover_command(param_value)
            elif param_name == "turn_angle":
                if param_value != 0:
                    success = self.reader_crtp.send_turn_command(param_value)
            elif param_name == "land_velocity":
                if param_value > 0:
                    success = self.reader_crtp.send_land_command()
            elif param_name == "move_distance":
                # move by given distance, blocking
                if param_value != 0:
                    self.reader_crtp.send_move_command(param_value)
                # move by given velocity, non-blocking
            elif param_name == "move_forward":
                if param_value != 0:
                    self.reader_crtp.send_forward_command(param_value)

            # send buzzer commands
            elif param_name == "buzzer_idx":
                if param_value >= 0:
                    self.get_logger().info(f"set {param_name} to {param_value}")
                    self.reader_crtp.send_buzzer_idx(param_value)
            elif param_name == "buzzer_effect":
                if param_value >= 0:
                    self.reader_crtp.send_buzzer_effect(param_value)
            elif param_name == "buzzer_freq":
                if param_value >= 0:
                    self.reader_crtp.send_buzzer_freq(param_value)

            elif param_name == "all":
                self.get_logger().info(f"set all motors to {param_value}")
                success = self.reader_crtp.send_thrust_command(param_value)
            elif param_name in [f"m{i}" for i in range(1, 5)]:
                self.get_logger().info(f"set {param_name} to {param_value}")
                success = self.reader_crtp.send_thrust_command(param_value, param_name)
            elif param_name == "send_audio_enable":
                self.reader_crtp.send_audio_enable(param_value)
            else:
                self.set_audio_param(param_name, param_value)
        if success: 
            return SetParametersResult(successful=True)
        else:
            return SetParametersResult(successful=False)

    def set_audio_param(self, param_name, param_value):
        try:
            self.reader_crtp.cf.param.set_value(f"audio.{param_name}", param_value)
            self.get_logger().info(
                f"changed {param_name} to {param_value}"
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
        publisher = Gateway(reader_crtp)
        print("done initializing")

        try:
            rclpy.spin(publisher)
            print("done spinning")

            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("node interrupted")
            reader_crtp.send_audio_enable(0)
            cf.param.set_value("motorPowerSet.enable", 0)
            reader_crtp.send_buzzer_freq(0)
            reader_crtp.send_buzzer_idx(0)
            print("stop buzzer, motors, and audio sending, wait for 1s...")
            time.sleep(1)
        except Exception:
            print("error occured")
            reader_crtp.send_audio_enable(0)
            cf.param.set_value("motorPowerSet.enable", 0)
            reader_crtp.send_buzzer_freq(0)
            reader_crtp.send_buzzer_idx(0)
            print("stop buzzer, motors, and audio sending, wait for 1s...")
            time.sleep(1)
            raise

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
