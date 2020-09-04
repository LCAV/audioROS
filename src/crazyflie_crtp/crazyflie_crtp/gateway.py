import logging
import os
import sys
import time

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir + "/../../../crazyflie-audio/python/")

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Pose, Point, Quaternion

import numpy as np

from audio_interfaces.msg import SignalsFreq, PoseRaw
from audio_stack.live_plotter import LivePlotter
from reader_crtp import ReaderCRTP

logging.basicConfig(level=logging.ERROR)
id = "radio://0/80/2M"
#id = "radio://0/48/2M"
#id = "radio://0/69/2M"

MAX_YLIM = 1e13  # set to inf for no effect.
MIN_YLIM = 1e-13  # set to -inf for no effect.

N_MICS = 4
FS = 32000
N = 1024

# Crazyflie audio parameters that can be set from here.
PARAMETERS_TUPLES = [
    ("debug", rclpy.Parameter.Type.INTEGER, 0),
    ("send_audio_enable", rclpy.Parameter.Type.INTEGER, 0),
    ("filter_snr_enable", rclpy.Parameter.Type.INTEGER, 0),
    ("filter_prop_enable", rclpy.Parameter.Type.INTEGER, 0),
    ("max_freq", rclpy.Parameter.Type.INTEGER, 10000),
    ("min_freq", rclpy.Parameter.Type.INTEGER, 100),
    ("delta_freq", rclpy.Parameter.Type.INTEGER, 100),
    ("use_iir", rclpy.Parameter.Type.INTEGER, 0),
    ("ma_window", rclpy.Parameter.Type.INTEGER, 0),
    ("alpha_iir", rclpy.Parameter.Type.DOUBLE, 0.5)
]


class Gateway(Node):
    def __init__(self, reader_crtp, mic_positions=None, plot=False):
        super().__init__("gateway")

        self.start_time = time.time()
        self.mic_positions = mic_positions

        self.publisher_signals = self.create_publisher(
            SignalsFreq, "audio/signals_f", 10
        )
        self.publisher_motion_pose = self.create_publisher(Pose, "motion/pose", 10)
        self.publisher_motion_pose_raw = self.create_publisher(
            PoseRaw, "motion/pose_raw", 10
        )
        self.plot = plot

        self.prev_position_x = 0
        self.prev_position_y = 0

        if self.plot:
            self.plotter = LivePlotter(MAX_YLIM, MIN_YLIM)
            self.plotter.ax.set_xlabel("angle [rad]")
            self.plotter.ax.set_ylabel("magnitude [-]")

        self.reader_crtp = reader_crtp

        parameters = []
        for param in PARAMETERS_TUPLES:
            self.declare_parameter(param[0])
            param_rclpy = rclpy.parameter.Parameter(*param)
            parameters.append(param_rclpy)
        self.set_parameters_callback(self.set_audio_params)
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
        self.get_logger().info(f"Publishing audio signals.")

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
        #self.get_logger().info(f"Read fbins: {fbins}")
        #self.get_logger().info(f"Read audio data: {signals_f_vect.reshape((8, 32))}")

        n_frequencies = len(fbins)
        assert n_frequencies == len(signals_f_vect) / (N_MICS * 2), \
            f"{n_frequencies} does not match {len(signals_f_vect)}"
        all_frequencies = np.fft.rfftfreq(n=N, d=1/FS)
        #frequencies = all_frequencies[:n_frequencies]
        frequencies = all_frequencies[fbins]

        signals_f = np.zeros((N_MICS, n_frequencies), dtype=np.complex128)
        for i in range(N_MICS):
            signals_f[i].real = signals_f_vect[i :: N_MICS * 2]
            signals_f[i].imag = signals_f_vect[i + N_MICS :: N_MICS * 2]


        # plot data
        if self.plot:
            labels = [f"mic{i}" for i in range(N_MICS)]
            self.plotter.update_lines(np.abs(signals_f), frequencies, labels=labels)

        # send data
        msg = SignalsFreq()
        msg.frequencies = [int(f) for f in frequencies]
        msg.signals_real_vect = list(signals_f.real.flatten())
        msg.signals_imag_vect = list(signals_f.imag.flatten())
        msg.timestamp = self.reader_crtp.audio_dict["timestamp"]
        msg.n_mics = N_MICS
        msg.n_frequencies = n_frequencies

        if self.mic_positions is not None:
            msg.mic_positions = list(self.mic_positions.flatten().astype(float))
        else:
            msg.mic_positions = []
        self.publisher_signals.publish(msg)

        self.get_logger().info(f"Published audio_dict.")

    def publish_motion_dict(self):
        self.get_logger().info(f"Publishing motion signals.")
        motion_dict = self.reader_crtp.motion_dict["data"]

        msg_pose_raw = PoseRaw()
        msg_pose_raw.dx = motion_dict["dx"]
        msg_pose_raw.dy = motion_dict["dy"]
        msg_pose_raw.dy = motion_dict["z"]
        msg_pose_raw.yaw = motion_dict["yaw"]
        msg_pose_raw.timestamp = self.reader_crtp.motion_dict["timestamp"]
        self.publisher_motion_pose_raw.publish(msg_pose_raw)

        # TODO(FD) fix the position update to consider also rotation.
        msg_pose = Pose()
        msg_pose.position = Point()
        msg_pose.position.x = motion_dict["dx"] + self.prev_position_x
        msg_pose.position.y = motion_dict["dy"] + self.prev_position_y
        msg_pose.position.z = motion_dict["z"]
        msg_pose.orientation = Quaternion()
        # TODO(FD): could replace this with tf.transformations or tf2.transformations
        from scipy.spatial.transform import Rotation

        r = Rotation.from_euler("z", motion_dict["yaw"], degrees=True)
        r_quat = r.as_quat()
        msg_pose.orientation.x = r_quat[0]
        msg_pose.orientation.y = r_quat[1]
        msg_pose.orientation.z = r_quat[2]
        msg_pose.orientation.w = r_quat[3]
        self.publisher_motion_pose.publish(msg_pose)
        self.get_logger().info("Published motion data.")

        self.prev_position_x = msg_pose.position.x
        self.prev_position_y = msg_pose.position.y

    def set_audio_params(self, params):
        for param in params:
            param_tuples = [p for p in PARAMETERS_TUPLES if p[0] == param.name]
            assert len(param_tuples) == 1
            if param_tuples[0][1] == rclpy.Parameter.Type.INTEGER:
                old_value = (
                    self.get_parameter(param.name).get_parameter_value().integer_value
                )
                new_value = param.get_parameter_value().integer_value
            elif param_tuples[0][1] == rclpy.Parameter.Type.DOUBLE:
                old_value = (
                    self.get_parameter(param.name).get_parameter_value().double_value
                )
                new_value = param.get_parameter_value().double_value
            else:
                raise ValueError(param_tuple)

            self.reader_crtp.cf.param.set_value(f"audio.{param.name}", new_value)
            self.get_logger().info(
                f"changing {param.name} from {old_value} to {new_value}"
            )
        return SetParametersResult(successful=True)


def main(args=None):
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    import cflib.crtp

    plot = False

    cflib.crtp.init_drivers(enable_debug_driver=False)
    rclpy.init(args=args)

    mic_d = 0.108  # distance between mics (meters)
    mic_positions = mic_d / 2 * np.c_[[1, 1], [1, -1], [-1, 1], [-1, -1]].T  #  #  #  #

    with SyncCrazyflie(id) as scf:
        cf = scf.cf
        # set_thrust(cf, 43000)
        reader_crtp = ReaderCRTP(cf, verbose=False)
        publisher = Gateway(reader_crtp, mic_positions=mic_positions, plot=plot)
        print("done initializing")

        try:
            rclpy.spin(publisher)
            print("done spinning")

            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("unset audio.send_audio_enable")
            cf.param.set_value("audio.send_audio_enable", 0)

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
