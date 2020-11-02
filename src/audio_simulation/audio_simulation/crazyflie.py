"""
crazyflie.py: Publish simulated audio signals for the Crazyflie drone in a room.  

"""

import copy as cp
import os
import sys
import math

import matplotlib.pyplot as plt
import numpy as np
import pyroomacoustics as pra
from scipy.spatial.transform import Rotation as R
from scipy.io import wavfile

import rclpy
from rclpy.node import Node

from audio_interfaces.msg import Signals
from audio_interfaces_py.messages import create_signals_message
from geometry_msgs.msg import Pose
from std_msgs.msg import String

sys.path.append(os.getcwd() + "/crazyflie-audio/python/")
from signals import generate_signal_random

# TODO(FD) make sure that this is consistent with the rest of the audio processing
# pipeline. Probably we want to add an extra package with the physical parameters
# of the audio deck, for example in "audio_description".  
N_MICS = 4  # number of microphones on Crazyflie
MIC_DISTANCE = 0.108  # distance between microphones
HEIGHT_MIC_ARRAY = 0.0 # height of mic array with respect to drone center (in meters)

ROOM_DIM = [2, 2, 2]  # room dimensions
SOURCE_POS = [1, 1, 1]  # source position
MAX_TIMESTAMP = 2**32 - 1  # max value of uint32
MAX_BUFFER = 2048  # number of samples in audio buffer
NUM_REFLECTIONS = 5 # number of reflections to consider in pyroomacoustis.
FS = 8000  # sampling frequency [Hz]


class AudioSimulation(Node):
    def __init__(self):
        super().__init__("audio_simulation")

        self.room = set_room(ROOM_DIM, MAX_BUFFER, [SOURCE_POS,], FS)
        self.time_idx = 0

        self.publisher_signals = self.create_publisher(Signals, "audio/signals", 10)
        self.subscription_position = self.create_subscription(
            Pose, "geometry/pose", self.pose_listener_callback, 10
        )

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

        microphones = generate_mic_position_array(rotation, drone_center)
        sim_room = self.simulation(microphones)

        # TODO(FD): for the moment, we send the first non-zero signal at this position, and 
        # we do not take into account the timestamp of the pose. In a more realistic simulation, 
        # we should send the signal that corresponds to the actual current time, given by 
        # the timestamp of the pose. 
        start_sample = starting_sample_nr(sim_room, microphones)

        signal_to_send = np.zeros((N_MICS, MAX_BUFFER), dtype=float)

        for i in range(N_MICS):
            signal_to_send[i] = sim_room.mic_array.signals[i][start_sample:start_sample+MAX_BUFFER]

        msg = create_signals_message(signal_to_send, microphones, self.time_idx, sim_room.fs)

        self.publisher_signals.publish(msg)
        self.get_logger().info(f"{self.time_idx}: Published audio signal")
        self.time_idx += 1

        if self.time_idx >= MAX_TIMESTAMP:
            self.get_logger().error("timestamp overflow")
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
        pyroom_cp = pra.ShoeBox(self.room.shoebox_dim)
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


def generate_mic_position_array(rotation, drone_center):
    """
    Calculate current mic postions based on the drone's pose. 
    """

    rot = R.from_quat(rotation)

    # TODO(FD) read this from audio_description package (see comment above). 
    mic_locs = MIC_DISTANCE / 2 * np.c_[[1, -1], [-1, -1], [1, 1], [-1, 1]].T # n_mics x 2
    mic_locs = np.c_[mic_locs, np.full(mic_locs.shape[0], HEIGHT_MIC_ARRAY)] 

    for i in range(mic_locs.shape[0]):
        mic_locs[i, :] = rot.apply(mic_locs[i, :] - drone_center) + drone_center

    return mic_locs


def main(args=None):
    rclpy.init(args=args)

    sim_node = AudioSimulation()

    rclpy.spin(sim_node)

    sim_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
