"""
crazyflie.py: Publish simulated audio signals for the Crazyflie drone in a room.  

"""

import os
import sys

import numpy as np
import pyroomacoustics as pra
from scipy.spatial.transform import Rotation as R
from scipy.io import wavfile

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose

from audio_interfaces.msg import Signals
from audio_interfaces_py.messages import create_signals_message
from crazyflie_description_py.parameters import MIC_POSITIONS, FS, N_BUFFER, N_MICS, HEIGHT_MIC_ARRAY

sys.path.append(os.getcwd() + "/crazyflie-audio/python/")
from signals import generate_signal_random


ROOM_DIM = [2, 2, 2]  # room dimensions
SOURCE_POS = [1, 1, 1]  # source position
MAX_TIMESTAMP = 2**32 - 1  # max value of uint32
NUM_REFLECTIONS = 5 # number of reflections to consider in pyroomacoustis.


class AudioSimulation(Node):
    def __init__(self):
        super().__init__("audio_simulation")

        self.room = set_room(ROOM_DIM, N_BUFFER, [SOURCE_POS,], FS)
        self.time_idx = 0

        self.publisher_signals = self.create_publisher(Signals, "audio/signals", 10)
        self.subscription_position = self.create_subscription(
            Pose, "geometry/pose", self.pose_listener_callback, 10
        )

        self.mic_positions = np.array(MIC_POSITIONS)


    def pose_listener_callback(self, msg_pose):
        """
        Run a simulation and send the audio signal in packets when a message is received 
        """

        rotation_rx = msg_pose.orientation
        drone_center_rx = msg_pose.position

        rotation = np.array(
            [rotation_rx.x, rotation_rx.y, rotation_rx.z, rotation_rx.w]
        )
        drone_center = np.array(
            [drone_center_rx.x, drone_center_rx.y, drone_center_rx.z]
        )

        mic_positions_global = global_mic_positions(self.mic_positions, rotation, drone_center)
        sim_room = self.simulation(mic_positions_global)

        # TODO(FD): for the moment, we send the first non-zero signal at this position, and 
        # we do not take into account the timestamp of the pose. In a more realistic simulation, 
        # we should send the signal that corresponds to the actual current time, given by 
        # the timestamp of the pose. 
        start_sample = starting_sample_nr(sim_room, mic_positions_global)

        signal_to_send = np.zeros((N_MICS, N_BUFFER), dtype=float)

        for i in range(N_MICS):
            signal_to_send[i] = sim_room.mic_array.signals[i][start_sample:start_sample+N_BUFFER]

        msg = create_signals_message(signal_to_send, self.mic_positions, self.time_idx, sim_room.fs)

        self.publisher_signals.publish(msg)

        self.get_logger().info(f"{self.time_idx}: Published audio signal")
        self.time_idx += 1

        if self.time_idx >= MAX_TIMESTAMP:
            self.get_logger().warn("timestamp overflow")
            self.time_idx = self.time_idx % MAX_TIMESTAMP


    def simulation(self, mic_array):
        """
        Run a pyroomacoustics simulation on a copy of the pyroom attribute of an AudioSimulation object.
        """
        pyroom_copy = self.copy_room()
        pyroom_copy.add_microphone_array(mic_array.T) # dim x n_mics
        pyroom_copy.simulate()
        return pyroom_copy


    def copy_room(self):
        """
        Copy the pyroom attribute of an AudioSimulation object.
        """
        pyroom_cp = pra.ShoeBox(self.room.shoebox_dim, fs=self.room.fs)
        for i in range(len(self.room.sources)):
            pyroom_cp.add_source(
                self.room.sources[i].position, signal=self.room.sources[i].signal
            )
        return pyroom_cp


def starting_sample_nr(pyroom, mic_array):
    """
    Calculate the index of the sample from which on all mics register audio from sources.
    """

    n_sources = pyroom.n_sources
    n_mics = mic_array.shape[0]
    dist_array = np.empty([n_mics, 1])
    tmp_dist = np.empty([n_sources, 1])

    for i in range(n_mics):
        for j in range(n_sources):
            tmp_dist[j] = np.linalg.norm(pyroom.sources[j].position - mic_array[i])
        dist_array[i] = np.max(tmp_dist)

    max_distance = np.max(dist_array)
    max_index = int(np.ceil((max_distance / pyroom.physics.get_sound_speed()) * pyroom.fs) + 1)
    return max_index


def set_room(room_dim, nr_samples, source_position_list, fs):
    """
    Set the shoe box room with specified dimensions, sampling frequency and sources
    """

    pyroom = pra.ShoeBox(room_dim, fs=fs, max_order=NUM_REFLECTIONS)

    # Generate a signal that is definitely long enough inside this room. 
    # The max possible delay comes from the main diagonal of cuboid. 
    max_delay_sec = np.sqrt(np.sum(np.array(room_dim)**2)) / pyroom.physics.get_sound_speed()
    duration_sec = nr_samples / fs + max_delay_sec * NUM_REFLECTIONS
    audio_source = generate_signal_random(fs, duration_sec)
    for i in range(len(source_position_list)):
        pyroom.add_source(source_position_list[i], signal=audio_source)
    return pyroom


def global_mic_positions(mic_positions, rotation, drone_center):
    """
    Calculate current mic postions based on the drone's pose. 
    """

    rot = R.from_quat(rotation)

    mic_positions_global = np.c_[mic_positions, np.full(mic_positions.shape[0], HEIGHT_MIC_ARRAY)] 

    for i in range(mic_positions.shape[0]):
        mic_positions_global[i, :] = rot.apply(mic_positions_global[i, :] - drone_center) + drone_center

    return mic_positions_global


def main(args=None):
    rclpy.init(args=args)

    sim_node = AudioSimulation()

    rclpy.spin(sim_node)

    sim_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
