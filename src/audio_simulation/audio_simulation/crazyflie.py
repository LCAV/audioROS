"""
crazyflie.py: Publish simulated audio signals for the Crazyflie drone in a room.  

"""

from enum import Enum
from math import atan2, ceil, degrees
import os
import sys
import time

import numpy as np
import pyroomacoustics as pra

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import Pose

from audio_interfaces.msg import Signals, PoseRaw
from audio_interfaces_py.messages import (
    create_signals_message,
    create_pose_raw_message_from_poses,
)
from audio_simulation.geometry import (
    global_positions_from_2d,
    global_positions_from_3d,
    get_relative_movement,
    get_starting_pose,
)
from audio_simulation.geometry import ROOM_DIM, SPEAKER_POSITION
from crazyflie_description_py.parameters import (
    MIC_POSITIONS,
    BUZZER_POSITION,
    FS,
    N_BUFFER,
    HEIGHT_MIC_ARRAY,
    HEIGHT_BUZZER,
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
    SIMULATE = 0
    PUBLISH = 1
    CHANGE_SIGNAL = 2
    UPDATE_GEOMETRY = 3
    IDLE = 3


MAX_TIMESTAMP = 2 ** 32 - 1  # max value of uint32
NUM_REFLECTIONS = 1  # number of reflections to consider in pyroomacoustis.
DIM = 2  # dimension of simulation
DURATION_SEC = 60  # duration of simulated audio signals
LOOP = True  # flag for looping the signal after reaching the end
NOISE = (
    1e-2  # white noise to add on signals (variance squared), set to None for no effect
)


def get_source_signal(source_type, source_freq):
    if source_type == "random":
        source_signal = generate_signal_random(FS, DURATION_SEC)
    elif source_type == "mono":
        source_signal = generate_signal_mono(FS, DURATION_SEC, frequency_hz=source_freq)
    elif source_type in [None, "none"]:
        source_signal = np.zeros(int(ceil(FS * DURATION_SEC)))
    else:
        raise ValueError(source_type)
    return source_signal


def get_starting_idx(pyroom, mic_array):
    """
    Calculate the index of the sample from which on all mics register
    audio from all sources.
    """
    # n_sources x dim:
    from audio_simulation.pyroom_helpers import get_max_delay
    return 1000

    p_sources = np.array([s.position for s in pyroom.sources if s.signal is not None])
    if len(p_sources) == 0:
        return 0
    assert p_sources.shape[1] == mic_array.shape[1]
    # n_sources x n_mics:
    distances = np.linalg.norm(p_sources[:, None, :] - mic_array[None, :, :], axis=2)
    max_delay = np.max(distances) / pyroom.physics.get_sound_speed()
    #max_delay = get_max_delay(pyroom)
    max_index = int(np.ceil(max_delay * FS))
    return 1000
    #return max_index


def create_room(speaker_position, buzzer_position, mic_array):
    """
    Create a shoe box room with two sources:
    - source 0: external sound source
    - source 1: buzzer sound source
    """
    assert mic_array.shape[1] in (2, 3), f"{mic_array.shape}"
    pyroom = pra.ShoeBox(
        ROOM_DIM[:DIM], fs=FS, max_order=NUM_REFLECTIONS, sigma2_awgn=NOISE
    )
    pyroom.add_source(speaker_position[:DIM])
    pyroom.add_source(buzzer_position[0, :DIM]) # 1 x 3
    pyroom.add_microphone_array(mic_array.T[:DIM, :])  # dim x n_mics
    return pyroom


class CrazyflieSimulation(Node):
    def __init__(self):
        super().__init__(
            "audio_simulation",
            automatically_declare_parameters_from_overrides=True,
            allow_undeclared_parameters=True,
        )
        self.global_idx = 0  # where we are currently in the signal
        self.time_idx = 0  # index current buffer
        self.end_idx = None  # length of signal

        self.n_simulation = N_BUFFER * 3  # number of samples to simulated at once

        self.publisher_pose_raw = self.create_publisher(
            PoseRaw, "geometry/pose_raw", 10
        )
        self.publisher_signals = self.create_publisher(Signals, "audio/signals", 10)
        self.subscription_position = self.create_subscription(
            Pose, "geometry/pose", self.pose_listener_callback, 10
        )

        self.create_timer(N_BUFFER / FS, self.timer_callback)

        self.mic_positions = np.array(MIC_POSITIONS) # 4 x 2
        self.buzzer_position = np.array(BUZZER_POSITION) # 1 x 2
        self.speaker_position_global = np.array(SPEAKER_POSITION) # 3,
        self.get_logger().warn(f"{self.mic_positions}")

        self.signals = None
        self.mic_positions_global = None
        self.buzzer_position_global = None
        self.buzzer_signal = None
        self.speaker_signal = None

        self.room = None
        self.current_pose = get_starting_pose()
        self.update_positions()
        self.room = self.create_room()

        # parameter stuff
        self.audio_params = {key: val[1] for key, val in PARAMS_DICT.items()}
        self.update_signals()

        self.set_parameters_callback(self.set_params)
        parameters = self.get_parameters(PARAMS_DICT.keys())
        self.set_parameters(parameters)

        self.state = State.CHANGE_SIGNAL

    def set_params(self, params):
        for param in params:
            # we need this in case this parameter
            # was not set at startup; then we use the default values.
            if param.type_ == param.Type.NOT_SET:
                param = rclpy.parameter.Parameter(param.name, *PARAMS_DICT[param.name])

            self.audio_params[param.name] = param.value

            if param.name in [
                "speaker_type",
                "speaker_freq",
                "buzzer_type",
                "buzzer_freq",
            ]:
                self.state = State.CHANGE_SIGNAL
        return SetParametersResult(successful=True)

    def create_room(self):
        return create_room(
            self.speaker_position_global,
            self.buzzer_position_global,
            self.mic_positions_global,
        )

    def update_signals(self):
        self.speaker_signal = get_source_signal(
            self.audio_params["speaker_type"], self.audio_params["speaker_freq"]
        )
        self.buzzer_signal = get_source_signal(
            self.audio_params["buzzer_type"], self.audio_params["buzzer_freq"]
        )
        self.end_idx = min(len(self.buzzer_signal), len(self.speaker_signal))

    def timer_callback(self):
        """
        Publish signal chunks at a rate defined by the timer.
        """
        self.get_logger().info(f"state: {self.state}")
        if self.state == State.CHANGE_SIGNAL:
            self.update_signals()
            self.state = State.SIMULATE
            return

        elif self.state == State.UPDATE_GEOMETRY:
            t1 = time.time()
            valid_positions = self.update_positions()
            # if drone is outside room, do nothing.
            if not valid_positions:
                return
            self.room = self.create_room()
            self.state = State.SIMULATE
            self.get_logger().warn(f"Updated geometry in {time.time() - t1:.2f}s")
            return

        elif self.state == State.SIMULATE:
            t1 = time.time()

            if self.global_idx == 0:
                self.global_idx = get_starting_idx(self.room, self.mic_positions_global)
                # alternative:
                # nonzero_indices = np.where(np.all(self.signals > 0, axis=0))[0]
                # self.global_idx = nonzero_indices[0]
                # self.end_idx = nonzero_indices[-1]
                self.get_logger().info(f"Starting at {self.global_idx}")

            self.simulate()
            self.global_idx += self.time_idx
            self.time_idx = 0

            self.get_logger().warn(f"Updated signals in {time.time() - t1:.2f}s")
            self.state = State.PUBLISH
            return

        elif self.state == State.PUBLISH:
            timestamp = self.get_time_ms()

            signal_to_send = self.signals[:, self.time_idx : self.time_idx + N_BUFFER]
            msg = create_signals_message(
                signal_to_send, self.mic_positions, timestamp, FS
            )
            self.publisher_signals.publish(msg)
            self.get_logger().info(f"{timestamp}: Published audio signal")

            self.time_idx += N_BUFFER

            if self.global_idx >= MAX_TIMESTAMP:
                self.get_logger().warn("timestamp overflow")

            elif (self.global_idx + N_BUFFER) >= self.end_idx:
                self.get_logger().warn("end of signal array")

            elif (self.time_idx + N_BUFFER) > self.n_simulation:
                self.state = State.SIMULATE
                return

            else:  # continue normally
                self.state = State.PUBLISH
                return

            # treat the cases where global_idx has reached limit
            if LOOP:
                self.get_logger().warn("looping")
                self.global_idx = 0  # will be filled next time we enter this function
                self.time_idx = 0
                self.state = State.SIMULATE
            else:
                self.get_logger().warn("exiting the node")
                self.destroy_node()
                rclpy.shutdown()  # exit the console command
            return

        if self.state == State.IDLE:
            return

    def get_time_ms(self):
        return int(round(self.global_idx / FS * 1000))

    def pose_listener_callback(self, msg_pose):
        """
        Run a simulation and update the signal content when a new pose is received.
        """
        delta = get_relative_movement(self.current_pose, msg_pose)
        # update the signals if this is the first measurement or if we have moved.
        if any(delta) or (self.current_pose is None):
            self.state = State.UPDATE_GEOMETRY
        else:
            self.get_logger().info("Did not move, not updating signals.")

        # create a new pose_raw message.
        pose_raw_msg = create_pose_raw_message_from_poses(self.current_pose, msg_pose)
        if SPEAKER_POSITION is not None:
            source_direction_deg = degrees(
                atan2(
                    SPEAKER_POSITION[1] - msg_pose.position.y,
                    SPEAKER_POSITION[0] - msg_pose.position.x,
                )
            )
            pose_raw_msg.source_direction_deg = source_direction_deg
        pose_raw_msg.timestamp = self.get_time_ms()
        self.publisher_pose_raw.publish(pose_raw_msg)

        self.current_pose = msg_pose
        return

    def simulate(self):
        """
        Run a pyroomacoustics simulation on a copy of the pyroom attribute 
        of an CrazyflieSimulation object.
        """
        from audio_simulation.pyroom_helpers import simulate_truncated

        if self.speaker_signal is None or self.buzzer_signal is None:
            self.update_signals()

        self.room.sources[0].signal = self.speaker_signal

        self.room.sources[1].signal = self.buzzer_signal
        self.signals = simulate_truncated(self.room, self.global_idx, self.n_simulation)

    def update_positions(self):
        mic_positions_global = global_positions_from_2d(
            self.mic_positions, self.current_pose, z=HEIGHT_MIC_ARRAY
        )[:, :DIM]
        buzzer_position_global = global_positions_from_2d(
            self.buzzer_position, self.current_pose, z=HEIGHT_BUZZER
        )[:, :DIM]
        for pos in mic_positions_global:
            if (self.room is not None) and not self.room.is_inside(pos):
                self.get_logger().warn(f"mic outside room: {pos}")
                return False
        if (self.room is not None) and not self.room.is_inside(buzzer_position_global):
            self.get_logger().warn(f"buzzer position outside room")
            return False
        self.mic_positions_global = mic_positions_global
        self.buzzer_position_global = buzzer_position_global
        return True


def main(args=None):
    rclpy.init(args=args)

    sim_node = CrazyflieSimulation()

    rclpy.spin(sim_node)

    sim_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
