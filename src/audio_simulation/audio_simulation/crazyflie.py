"""
crazyflie.py: Publish simulated audio signals for the Crazyflie drone in a room.  

"""

from enum import Enum
from math import ceil
import os
import sys
import time

import numpy as np
import pyroomacoustics as pra

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import PoseStamped

from audio_interfaces.msg import Signals
from audio_interfaces_py.messages import create_signals_message
from audio_simulation.geometry import (
    global_positions_from_2d,
    get_relative_movement,
)
from crazyflie_description_py.parameters import (
    MIC_POSITIONS,
    BUZZER_POSITION,
    FS,
    N_BUFFER,
    HEIGHT_MIC_ARRAY,
    HEIGHT_BUZZER,
)
from crazyflie_description_py.experiments import (
    get_starting_pose_msg,
    SPEAKER_POSITION,
    ROOM_DIM,
)

sys.path.append(os.getcwd() + "/crazyflie-audio/python/")
from signals import generate_signal_random, generate_signal_mono

PARAMS_DICT = {
    "speaker_type": (rclpy.Parameter.Type.STRING, "mono"),
    "speaker_freq": (rclpy.Parameter.Type.INTEGER, 3500),
    "buzzer_type": (rclpy.Parameter.Type.STRING, "mono"),
    "buzzer_freq": (rclpy.Parameter.Type.INTEGER, 4000),
}


class State(Enum):
    SIMULATE = 1
    PUBLISH = 2
    UPDATE_SIGNALS = 3
    UPDATE_GEOMETRY = 4


MAX_TIMESTAMP = 2 ** 32 - 1  # max value of uint32
NUM_REFLECTIONS = 1  # number of reflections to consider in pyroomacoustis.
DIM = 2  # dimension of simulation
DURATION_SEC = 20  # duration of simulated audio signals
LOOP = True  # flag for looping the signal after reaching the end
NOISE = 1e-2  # white noise to add on signals (variance squared), set to None for no effect
STARTING_INDEX = 3000  # start reading the signal at this index instead of zero (to avoid boundary effects)


def get_source_signal(source_type, source_freq):
    if source_type == "random":
        source_signal = generate_signal_random(FS, DURATION_SEC)
    elif source_type == "mono":
        source_signal = generate_signal_mono(
            FS, DURATION_SEC, frequency_hz=source_freq
        )
    elif source_type in [None, "none"]:
        source_signal = np.zeros(int(ceil(FS * DURATION_SEC)))
    else:
        raise ValueError(source_type)
    return source_signal


def create_room(speaker_position, buzzer_position, mic_array):
    """
    Create a shoe box room with two sources:
    - source 0: external sound source
    - source 1: buzzer sound source
    """
    assert speaker_position.ndim == 1
    assert buzzer_position.ndim == 2
    assert mic_array.ndim == 2
    assert mic_array.shape[1] in (2, 3), f"{mic_array.shape}"

    pyroom = pra.ShoeBox(
        ROOM_DIM[:DIM], fs=FS, max_order=NUM_REFLECTIONS, sigma2_awgn=NOISE
    )
    pyroom.add_source(speaker_position[:DIM])  #
    pyroom.add_source(buzzer_position[0, :DIM])  # 1 x 2
    pyroom.add_microphone_array(mic_array.T[:DIM, :])  # dim x n_mics
    return pyroom


class CrazyflieSimulation(Node):
    # constants
    mic_positions = np.array(MIC_POSITIONS)  # 4 x 2
    buzzer_position = np.array(BUZZER_POSITION)  # 1 x 2
    speaker_position_global = np.array(SPEAKER_POSITION)  # 3,

    def __init__(self):
        super().__init__(
            "audio_simulation",
            automatically_declare_parameters_from_overrides=True,
            allow_undeclared_parameters=True,
        )
        self.publisher_signals = self.create_publisher(
            Signals, "audio/signals", 10
        )
        self.subscription_position = self.create_subscription(
            PoseStamped, "geometry/pose", self.pose_listener_callback, 10
        )
        self.create_timer(N_BUFFER / FS, self.timer_callback)

        # parameters
        self.simulation_idx = (
            STARTING_INDEX  # where we are currently in the signal
        )
        self.buffer_idx = 0  # index current buffer
        self.end_idx = None  # length of signal

        self.n_buffers = 3  # number of buffers to simulated at once

        self.mic_positions_global = None
        self.buzzer_position_global = None
        self.mic_signals = None
        self.buzzer_signal = None
        self.speaker_signal = None

        self.room = None
        self.current_pose = get_starting_pose_msg().pose

        # parameter stuff
        self.audio_params = {key: val[1] for key, val in PARAMS_DICT.items()}
        self.set_parameters_callback(self.set_params)
        parameters = self.get_parameters(PARAMS_DICT.keys())
        self.set_parameters(parameters)

        # start
        self.update_positions()
        self.update_geometry()
        self.update_source_signals()
        self.state = State.SIMULATE

    def set_params(self, params):
        for param in params:
            # we need this in case this parameter
            # was not set at startup; then we use the default values.
            if param.type_ == param.Type.NOT_SET:
                param = rclpy.parameter.Parameter(
                    param.name, *PARAMS_DICT[param.name]
                )

            self.audio_params[param.name] = param.value

            if param.name in [
                "speaker_type",
                "speaker_freq",
                "buzzer_type",
                "buzzer_freq",
            ]:
                self.state = State.UPDATE_SIGNALS
        return SetParametersResult(successful=True)

    def timer_callback(self):
        """
        Publish signal chunks at a rate defined by the timer.
        """
        self.get_logger().info(f"state: {self.state}")
        if self.state == State.UPDATE_SIGNALS:

            self.update_source_signals()
            self.update_mic_signals()

            self.state = State.PUBLISH
            return

        elif self.state == State.UPDATE_GEOMETRY:
            t1 = time.time()
            valid_positions = self.update_positions()
            # if drone is outside room, do nothing.
            if not valid_positions:
                return

            self.update_geometry()
            self.update_mic_signals()

            self.state = State.PUBLISH
            self.get_logger().info(
                f"Updated geometry in {time.time() - t1:.2f}s"
            )
            return

        elif self.state == State.SIMULATE:
            t1 = time.time()

            self.update_mic_signals()

            self.get_logger().info(
                f"Updated signals in {time.time() - t1:.2f}s"
            )
            self.state = State.PUBLISH
            return

        elif self.state == State.PUBLISH:
            timestamp = self.get_time_ms()

            assert (self.buffer_idx + 1) * N_BUFFER <= self.mic_signals.shape[
                1
            ], f"not enough values {self.mic_signals.shape} for buffer idx {self.buffer_idx} {N_BUFFER}"
            signal_to_send = self.mic_signals[
                :, self.buffer_idx * N_BUFFER : (self.buffer_idx + 1) * N_BUFFER
            ]
            msg = create_signals_message(
                signal_to_send, CrazyflieSimulation.mic_positions, timestamp, FS
            )
            self.publisher_signals.publish(msg)
            self.get_logger().info(f"{timestamp}: Published audio signal")

            self.buffer_idx += 1

            if self.buffer_idx >= self.n_buffers:
                self.simulation_idx += self.buffer_idx * N_BUFFER
                self.buffer_idx = 0

                if self.get_time_ms() >= MAX_TIMESTAMP:
                    self.get_logger().warn("Timestamp overflow.")

                elif (
                    self.simulation_idx + N_BUFFER * self.n_buffers
                ) >= self.end_idx:
                    self.get_logger().warn("End of signal array.")

                else:
                    self.state = State.SIMULATE
                    return
            else:  # continue normally
                self.state = State.PUBLISH
                return

            # treat the cases where simulation_idx has reached limit
            if LOOP:
                self.get_logger().warn("Loop back to beginning.")
                self.simulation_idx = STARTING_INDEX
                self.buffer_idx = 0
                self.state = State.SIMULATE
            else:
                self.get_logger().warn("Exiting node.")
                self.destroy_node()
                rclpy.shutdown()  # exit the console command
            return

    def pose_listener_callback(self, msg_pose):
        """
        Run a simulation and update the signal content when a new pose is received.
        """
        delta = get_relative_movement(self.current_pose, msg_pose.pose)

        # update the room geometry if we have moved.
        if any(delta):
            self.state = State.UPDATE_GEOMETRY
        else:
            self.get_logger().info("Did not move, not updating geometry.")
        self.current_pose = msg_pose.pose
        return

    def update_geometry(self):
        self.room = create_room(
            CrazyflieSimulation.speaker_position_global,
            self.buzzer_position_global,
            self.mic_positions_global,
        )

    def update_mic_signals(self):
        """
        Run a pyroomacoustics simulation on a copy of the pyroom attribute
        of an CrazyflieSimulation object.
        """
        from audio_simulation.pyroom_helpers import simulate_truncated

        self.room.sources[0].signal = self.speaker_signal
        self.room.sources[1].signal = self.buzzer_signal

        assert (
            len(self.speaker_signal)
            >= self.simulation_idx + self.n_buffers * N_BUFFER
        )
        self.mic_signals = simulate_truncated(
            self.room, self.simulation_idx, self.n_buffers * N_BUFFER
        )
        self.get_logger().info(f"updated signals: {self.mic_signals.shape}")

    def update_source_signals(self):
        self.speaker_signal = get_source_signal(
            self.audio_params["speaker_type"], self.audio_params["speaker_freq"]
        )
        self.buzzer_signal = get_source_signal(
            self.audio_params["buzzer_type"], self.audio_params["buzzer_freq"]
        )
        self.end_idx = min(len(self.buzzer_signal), len(self.speaker_signal))

    def update_positions(self):
        mic_positions_global = global_positions_from_2d(
            CrazyflieSimulation.mic_positions,
            self.current_pose,
            z=HEIGHT_MIC_ARRAY,
        )[:, :DIM]

        buzzer_position_global = global_positions_from_2d(
            CrazyflieSimulation.buzzer_position,
            self.current_pose,
            z=HEIGHT_BUZZER,
        )[:, :DIM]

        for pos in [pos for pos in mic_positions_global] + [
            buzzer_position_global
        ]:
            if (self.room is not None) and not self.room.is_inside(pos):
                self.get_logger().warn(f"Position outside room: {pos}")
                return False

        self.mic_positions_global = mic_positions_global
        self.buzzer_position_global = buzzer_position_global
        return True

    def get_time_ms(self):
        # use one of below?
        # self.get_clock().now()
        # self.get_clock().now().seconds_nanoseconds()
        # time.time()
        return int(
            round(
                (self.simulation_idx + self.buffer_idx * N_BUFFER) / FS * 1000
            )
        )


def main(args=None):
    rclpy.init(args=args)

    sim_node = CrazyflieSimulation()

    rclpy.spin(sim_node)

    sim_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
