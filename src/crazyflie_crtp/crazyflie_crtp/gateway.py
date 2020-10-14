import logging
import os
import sys
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Pose 

from audio_interfaces.msg import SignalsFreq, PoseRaw
from audio_interfaces_py.messages import create_signals_freq_message, create_pose_message, create_pose_raw_message
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir + "/../../../crazyflie-audio/python/")
from reader_crtp import ReaderCRTP

logging.basicConfig(level=logging.ERROR)
id = "radio://0/80/2M"
#id = "radio://0/48/2M"
#id = "radio://0/69/2M"

MAX_YLIM = 1e13  # set to inf for no effect.
MIN_YLIM = 1e-13  # set to -inf for no effect.

N_MICS = 4
FS = 32000
N = 2048

# Crazyflie audio parameters that can be set from here.
AUDIO_PARAMETERS_TUPLES = [
    ("debug", rclpy.Parameter.Type.INTEGER, 0),
    ("send_audio_enable", rclpy.Parameter.Type.INTEGER, 1),
    ("min_freq", rclpy.Parameter.Type.INTEGER, 200),
    ("max_freq", rclpy.Parameter.Type.INTEGER, 7000),
    ("delta_freq", rclpy.Parameter.Type.INTEGER, 100),
    ("n_average", rclpy.Parameter.Type.INTEGER, 5),
    ("filter_snr_enable", rclpy.Parameter.Type.INTEGER, 0),
    ("filter_prop_enable", rclpy.Parameter.Type.INTEGER, 0),
]

MOTOR_PARAMETERS_TUPLES = [
    ("all", rclpy.Parameter.Type.INTEGER, 0),
    ("m1", rclpy.Parameter.Type.INTEGER, 0),
    ("m2", rclpy.Parameter.Type.INTEGER, 0),
    ("m3", rclpy.Parameter.Type.INTEGER, 0),
    ("m4", rclpy.Parameter.Type.INTEGER, 0),
    ("enable", rclpy.Parameter.Type.INTEGER, 0),
]


class Gateway(Node):
    def __init__(self, reader_crtp, mic_positions=None):
        super().__init__("gateway")

        self.start_time = time.time()
        self.mic_positions = mic_positions

        self.publisher_signals = self.create_publisher(
            SignalsFreq, "audio/signals_f", 10
        )
        self.publisher_motion_pose = self.create_publisher(Pose, "geometry/pose", 10)
        self.publisher_motion_pose_raw = self.create_publisher(
            PoseRaw, "geometry/pose_raw", 10
        )

        self.prev_position_x = 0.0
        self.prev_position_y = 0.0

        self.reader_crtp = reader_crtp

        parameters = []

        for param in AUDIO_PARAMETERS_TUPLES:
            self.declare_parameter(param[0])
            param_rclpy = rclpy.parameter.Parameter(*param)
            parameters.append(param_rclpy)

        for param in MOTOR_PARAMETERS_TUPLES:
            self.declare_parameter(param[0])
            param_rclpy = rclpy.parameter.Parameter(*param)
            parameters.append(param_rclpy)

        self.set_parameters_callback(self.set_params)
        self.set_parameters(parameters)

        # choose high publish rate so that we introduce as little
        # waiting time as possible
        self.create_timer(0.001, self.publish_current_data)

    def publish_current_data(self):
        if not self.reader_crtp.audio_dict["published"]:
            self.publish_audio_dict()
            self.reader_crtp.audio_dict["published"] = True

        if not self.reader_crtp.motion_dict["published"]:
            self.publish_motion_dict()
            self.reader_crtp.motion_dict["published"] = True

    def publish_audio_dict(self):
        # read audio
        signals_f_vect = self.reader_crtp.audio_dict["data"]
        if signals_f_vect is None:
            self.get_logger().warn("Empty audio. Not publishing")
            return

        # read frequencies
        if self.reader_crtp.fbins_dict["published"]:
            self.get_logger().error("Synchronization issue: already published fbins")
        fbins = self.reader_crtp.fbins_dict["data"]
        if fbins is None:
            self.get_logger().warn("Empty fbins. Not publishing")
            return

        #This seems to be ok now
        #self.get_logger().info(f"Read audio data: {signals_f_vect.reshape((8, 32))}")

        n_frequencies = len(fbins)
        assert n_frequencies == len(signals_f_vect) / (N_MICS * 2), \
            f"{n_frequencies} does not match {len(signals_f_vect)}"
        all_frequencies = np.fft.rfftfreq(n=N, d=1/FS)
        #frequencies = all_frequencies[:n_frequencies]
        try:
            assert np.any(fbins>0)
            if len(set(fbins)) < len(fbins):
                #self.get_logger().warn(f"Duplicate values in fbins! unique values:{len(set(fbins))}")
                return
            frequencies = all_frequencies[fbins]
        except:
            #self.get_logger().warn(f"Ignoring fbins: {fbins[:5]}")
            return

        signals_f = np.zeros((N_MICS, n_frequencies), dtype=np.complex128)
        for i in range(N_MICS):
            signals_f[i].real = signals_f_vect[i :: N_MICS * 2]
            signals_f[i].imag = signals_f_vect[i + N_MICS :: N_MICS * 2]

        abs_signals_f = np.abs(signals_f)
        if np.any(abs_signals_f[:3, :] > 1e5) or np.any(abs_signals_f[:3, :] < 1e-10):
            #self.get_logger().warn(f"Ignoring audio: {abs_signals_f.flatten()[:5]}")
            return

        msg = create_signals_freq_message(signals_f.T, frequencies, self.mic_positions, 
                self.reader_crtp.audio_dict["timestamp"], FS)
        self.publisher_signals.publish(msg)

        self.get_logger().info(f"{msg.timestamp}: Published audio data with fbins {fbins[[0, 1, 2, -1]]}")

    def publish_motion_dict(self):
        motion_dict = self.reader_crtp.motion_dict["data"]
        timestamp = self.reader_crtp.motion_dict["timestamp"]

        msg_pose_raw = create_pose_raw_message(motion_dict, timestamp)
        self.publisher_motion_pose_raw.publish(msg_pose_raw)

        msg_pose = create_pose_message(motion_dict, 
                self.prev_position_x, self.prev_position_y, timestamp)
        self.publisher_motion_pose.publish(msg_pose)

        self.prev_position_x = msg_pose.position.x
        self.prev_position_y = msg_pose.position.y
        self.get_logger().debug(f"{msg_pose_raw.timestamp}: Published motion data.")

    def set_params(self, params):
        for param in params: 
            param_tuples_audio = [p for p in AUDIO_PARAMETERS_TUPLES if p[0] == param.name]
            param_tuples_motor = [p for p in MOTOR_PARAMETERS_TUPLES if p[0] == param.name]

            if len(param_tuples_audio) == 1:
                self.set_param(param, param_tuples_audio[0], "audio")
            elif len(param_tuples_motor) == 1:
                # TODO(FD) find more elegant way to do this
                #for motor in [f"m{i}" for i in range(1, 5)]:
                #    param = self.get_parameter(motor)
                value = param.get_parameter_value().integer_value
                if param.name == "all":
                    [self.reader_crtp.cf.param.set_value(f"motorPowerSet.m{i}", value) for i in range(1, 5)]
                    self.get_logger().info( f"changing all motors to {value}")
                else:
                    self.reader_crtp.cf.param.set_value(f"motorPowerSet.{param.name}", value)
                    self.get_logger().info(f"changing {param.name} to {value}")
                if value > 0:
                    print("changing motorPowerSet.enable to 1")
                    self.reader_crtp.cf.param.set_value("motorPowerSet.enable", 1)
            else:
                raise ValueError(param)
        return SetParametersResult(successful=True)

    def set_param(self, param, param_tuple, param_class):
        if param_tuple[1] == rclpy.Parameter.Type.INTEGER:
            old_value = (
                self.get_parameter(param.name).get_parameter_value().integer_value
            )
            new_value = param.get_parameter_value().integer_value
        elif param_tuple[1] == rclpy.Parameter.Type.DOUBLE:
            old_value = (
                self.get_parameter(param.name).get_parameter_value().double_value
            )
            new_value = param.get_parameter_value().double_value
        else:
            raise ValueError(param_tuple)

        self.reader_crtp.cf.param.set_value(f"{param_class}.{param.name}", new_value)
        self.get_logger().info(
            f"changing {param.name} from {old_value} to {new_value}"
        )
        return 



def main(args=None):
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    import cflib.crtp

    verbose = False
    log_motion = True # get position logging from Crazyflie.

    cflib.crtp.init_drivers(enable_debug_driver=False)
    rclpy.init(args=args)

    mic_d = 0.108  # distance between mics (meters)
    mic_positions = mic_d / 2 * np.c_[[1, 1], [1, -1], [-1, 1], [-1, -1]].T

    with SyncCrazyflie(id) as scf:
        cf = scf.cf
        # set_thrust(cf, 43000)
        reader_crtp = ReaderCRTP(cf, verbose=verbose, log_motion=log_motion)
        publisher = Gateway(reader_crtp, mic_positions=mic_positions)
        print("done initializing")

        try:
            rclpy.spin(publisher)
            print("done spinning")

            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            cf.param.set_value("audio.send_audio_enable", 0)
            cf.param.set_value("motorPowerSet.enable", 0)
            print("reset audio.send_audio_enable and motorPowerSet.enable, wait for 1s...")
            time.sleep(1)

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
