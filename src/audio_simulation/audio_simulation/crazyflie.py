"""
crazyflie.py: Publish simulated audio signals for the Crazyflie drone in a room.  

"""

import os
import sys

import numpy as np
import pyroomacoustics as pra
from scipy.spatial.transform import Rotation as R
from math import atan2, degrees

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose

from audio_interfaces.msg import Signals, PoseRaw
from audio_interfaces_py.messages import create_signals_message
from crazyflie_description_py.parameters import MIC_POSITIONS, FS, N_BUFFER, N_MICS, HEIGHT_MIC_ARRAY

sys.path.append(os.getcwd() + "/crazyflie-audio/python/")
from signals import generate_signal_random


ROOM_DIM = [2, 2, 2]  # room dimensions
SOURCE_POS = [1, 1, 1]  # source position
MAX_TIMESTAMP = 2**32 - 1  # max value of uint32
NUM_REFLECTIONS = 5 # number of reflections to consider in pyroomacoustis.
LOOP = False # flag for looping the signal after reaching the end


class AudioSimulation(Node):
    def __init__(self):
        super().__init__("audio_simulation")

        self.room = set_room(ROOM_DIM, N_BUFFER, [SOURCE_POS,], FS)
        self.time_idx = 0

        self.publisher_rawpose = self.create_publisher(PoseRaw, "geometry/pose_raw", 10)
        self.publisher_signals = self.create_publisher(Signals, "audio/signals", 10)
        self.subscription_position = self.create_subscription(
            Pose, "geometry/pose", self.pose_listener_callback, 10
        )

        self.create_timer(N_BUFFER/FS, self.constant_publisher)

        #TODO: there has to be a better way to initialize it
        self.signals = []
        self.mic_positions_global = []
        self.first_sample = 0
        self.current_pose = Pose()
        self.mic_positions = np.array(MIC_POSITIONS)
   

    def constant_publisher(self):
        """
        Publish signal chunks at a rate defined by the timer
        """
        
        
        if len(self.signals) == 0:
            return 
        
        if self.time_idx == 0:
            self.time_idx = starting_sample_nr(self.room, self.mic_positions_global)
            self.first_sample = self.time_idx 

        if self.time_idx + N_BUFFER < self.signals.shape[1]:
            signal_to_send = np.zeros((N_MICS, N_BUFFER), dtype=float)
        else:
            signal_to_send = np.zeros((N_MICS, (self.signals.shape[1] - self.first_sample) % N_BUFFER), dtype=float)

        for i in range(N_MICS):
            signal_to_send[i] = self.signals[i][self.time_idx:self.time_idx+N_BUFFER]

        msg = create_signals_message(signal_to_send, self.mic_positions, self.time_idx, self.room.fs)
        self.publisher_signals.publish(msg)
        self.get_logger().info(f"{self.time_idx}: Published audio signal")
        self.time_idx += N_BUFFER
       
        if self.time_idx >= MAX_TIMESTAMP:
            self.get_logger().warn("timestamp overflow")
            if LOOP:
                self.time_idx = starting_sample_nr(self.room, self.mic_positions_global)
                self.first_sample = self.time_idx 
            else:
                self.get_logger().warn("exiting the node")
                self.destroy_node()
                rclpy.shutdown()
        
        if self.time_idx >= self.signals.shape[1]:
            if LOOP:
                self.get_logger().warn("end of signal array - starting sending from the beginning")
                self.time_idx = starting_sample_nr(self.room, self.mic_positions_global)
                self.first_sample = self.time_idx 
            else:
                self.get_logger().warn("end of signal array - exiting the node")
                self.destroy_node()
                rclpy.shutdown()


    def pose_listener_callback(self, msg_pose):
        """
        Run a simulation, update a signal content and publish a PoseRaw() message  when a new pose is received 
        """

        rotation_rx = msg_pose.orientation
        drone_center_rx = msg_pose.position

        rotation = np.array(
            [rotation_rx.x, rotation_rx.y, rotation_rx.z, rotation_rx.w]
        )
        drone_center = np.array(
            [drone_center_rx.x, drone_center_rx.y, drone_center_rx.z]
        )

        self.mic_positions_global = global_mic_positions(self.mic_positions, rotation, drone_center)
        sim_room = self.simulation(self.mic_positions_global)
        self.signals = sim_room.mic_array.signals
        
        # TODO: timestamp
        pose_raw = create_pose_raw_message(self.current_pose, msg_pose, 0)
        self.current_pose = msg_pose

        self.publisher_rawpose.publish(pose_raw)
        self.get_logger().info("Published pose raw")


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


def create_pose_raw_message(previous_pose, current_pose, timestamp):
    """
    Create a message of type PoseRaw() based on previous and current pose
    """
    
    prev_position = previous_pose.position 
    curr_position = current_pose.position 
    curr_orientation = current_pose.orientation 
    
    pose_raw = PoseRaw()
    pose_raw.timestamp = timestamp
    pose_raw.dx = curr_position.x - prev_position.x
    pose_raw.dy = curr_position.y - prev_position.y
    pose_raw.z = curr_position.z
    pose_raw.yaw_deg = get_yaw(curr_orientation)
    pose_raw.yaw_rate_deg = 0.0
    pose_raw.source_direction_deg = degrees(atan2(curr_position.y, curr_position.x))

    return pose_raw


def get_yaw(quaternion):
    """
    Calculate yaw from a quaternion
    """

    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    yaw = degrees(atan2(2.0 * (y*z + w*x), w*w -  x*x - y*y + z*z))
    
    return yaw


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
