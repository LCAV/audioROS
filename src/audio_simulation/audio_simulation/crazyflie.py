"""
crazyflie.py: Publish simulated audio signals for the Crazyflie drone in a room.  

"""

import os
import sys

import numpy as np
import pyroomacoustics as pra
from math import atan2, degrees

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose

from audio_interfaces.msg import Signals, PoseRaw
from audio_interfaces_py.messages import create_signals_message, create_pose_raw_message_from_poses
from audio_simulation.geometry import global_positions, get_relative_movement
from crazyflie_description_py.parameters import MIC_POSITIONS, BUZZER_POSITION, FS, N_BUFFER, HEIGHT_MIC_ARRAY, HEIGHT_BUZZER

sys.path.append(os.getcwd() + "/crazyflie-audio/python/")
from signals import generate_signal_random, generate_signal_mono


ROOM_DIM = [7, 9, 4]  # room dimensions [m].
SOURCE_POS = [5, 8, 1]  # source position [m], None for no external source.
BUZZER_ON = True # add buzzer to drone.
BUZZER_FREQ = 4000 # Hz, 
MAX_TIMESTAMP = 2**32 - 1  # max value of uint32
NUM_REFLECTIONS = 1 # number of reflections to consider in pyroomacoustis.
DURATION_SEC = 10 # duration of simulated audio signal 
LOOP = True # flag for looping the signal after reaching the end


class CrazyflieSimulation(Node):
    def __init__(self):
        super().__init__("audio_simulation")

        self.room = create_room(ROOM_DIM, [SOURCE_POS,], FS)
        self.time_idx = 0

        self.publisher_pose_raw = self.create_publisher(PoseRaw, "geometry/pose_raw", 10)
        self.publisher_signals = self.create_publisher(Signals, "audio/signals", 10)
        self.subscription_position = self.create_subscription(
            Pose, "geometry/pose", self.pose_listener_callback, 10
        )

        self.create_timer(N_BUFFER/FS, self.signals_publisher)

        self.signals = []
        self.mic_positions_global = []
        self.current_pose = None
        self.mic_positions = np.array(MIC_POSITIONS)
        self.buzzer_position = np.array(BUZZER_POSITION)
        self.pose_raw_msg = None
   

    def signals_publisher(self):
        """
        Publish signal chunks at a rate defined by the timer.
        """
        if len(self.signals) == 0:
            return 
        
        if self.time_idx == 0:
            self.time_idx = starting_sample_nr(self.room, self.mic_positions_global)

        assert (self.time_idx + N_BUFFER) < self.signals.shape[1]

        timestamp = self.get_time_ms() 
        signal_to_send = self.signals[:, self.time_idx:self.time_idx+N_BUFFER]

        msg = create_signals_message(signal_to_send, self.mic_positions, timestamp, self.room.fs)
        self.publisher_signals.publish(msg)

        if self.pose_raw_msg is not None:
            self.pose_raw_msg.timestamp = timestamp
            self.publisher_pose_raw.publish(self.pose_raw_msg)
        self.get_logger().info(f"{timestamp}: Published audio and pose signal")

        self.time_idx += N_BUFFER
       
        if self.time_idx >= MAX_TIMESTAMP:
            self.get_logger().warn("timestamp overflow")
        elif (self.time_idx + N_BUFFER) >= self.signals.shape[1]:
            self.get_logger().warn("end of signal array")
        else: # continue normally
            return 

        # treat the cases where time_idx has reached limit
        if LOOP:
            self.time_idx = 0 # will be filled next time we enter this function
        else:
            self.get_logger().warn("exiting the node")
            self.destroy_node()
            # shutdown => exiting the console command
            rclpy.shutdown()


    def get_time_ms(self):
        return int(round(self.time_idx / FS * 1000))


    def pose_listener_callback(self, msg_pose):
        """
        Run a simulation, update the signal content and publish a PoseRaw
        message  when a new pose is received.
        """
        self.mic_positions_global = global_positions(self.mic_positions, msg_pose, z=HEIGHT_MIC_ARRAY)

        if SOURCE_POS is not None:
            source_direction_deg = degrees(atan2(
                    SOURCE_POS[1] - msg_pose.position.y, 
                    SOURCE_POS[0] - msg_pose.position.x
                )
            )
        else:
            source_direction_deg = 0

        delta = None
        if self.current_pose is not None:
            self.pose_raw_msg = create_pose_raw_message_from_poses(self.current_pose, msg_pose)

            self.pose_raw_msg.source_direction_deg = source_direction_deg 
            delta = get_relative_movement(self.current_pose, msg_pose)

        # update the signals if this is the first 
        # measurement or if we have moved.
        if (delta is None) or any(delta):

            buzzer = None
            if BUZZER_ON: 
                buzzer = global_positions(self.buzzer_position, msg_pose, z=HEIGHT_BUZZER)

            self.room = self.simulation(self.mic_positions_global, buzzer=buzzer)
            self.signals = self.room.mic_array.signals

        elif (delta is not None):
            self.get_logger().info("Did not move, not updating signals.")

        self.current_pose = msg_pose


    def simulation(self, mic_array, buzzer=None):
        """
        Run a pyroomacoustics simulation on a copy of the pyroom attribute 
        of an CrazyflieSimulation object.
        """
        pyroom_copy = create_room(ROOM_DIM, [SOURCE_POS,], FS)

        pyroom_copy.add_microphone_array(mic_array.T) # dim x n_mics
        if buzzer is not None: 
            pyroom_copy.sources[-1].position = buzzer.flatten()

        print('simulating with:')
        print('sources', [s.position for s in pyroom_copy.sources])
        print('mics:', pyroom_copy.mic_array.R)

        pyroom_copy.simulate()
        return pyroom_copy


def starting_sample_nr(pyroom, mic_array):
    """
    Calculate the index of the sample from which on all mics register 
    audio from all sources.
    """
    # n_sources x 3:
    p_sources = np.array([s.position for s in pyroom.sources if s.position is not None]) 
    # n_sources x n_mics:
    distances = np.linalg.norm(p_sources[:, None, :] - mic_array[None, :], axis=2)
    max_distance = np.max(distances)
    max_index = int(np.ceil((max_distance / pyroom.physics.get_sound_speed()) * pyroom.fs) + 1)
    return max_index


def create_room(room_dim, source_position_list, fs, duration_sec=DURATION_SEC):
    """
    Set the shoe box room with specified dimensions, 
    sampling frequency and sources.
    """
    pyroom = pra.ShoeBox(room_dim, fs=fs, max_order=NUM_REFLECTIONS)

    # The max possible delay comes from the main diagonal of cuboid. 
    max_delay_sec = np.sqrt(np.sum(np.array(room_dim)**2)) / pyroom.physics.get_sound_speed()
    assert duration_sec > max_delay_sec

    for source in source_position_list:
        if source is not None:
            audio_source = generate_signal_random(fs, duration_sec)
            pyroom.add_source(source, signal=audio_source)

    if BUZZER_ON: 
        buzzer = [0, 0, 0] # will be overwritten
        buzzer_signal = generate_signal_mono(fs, duration_sec, frequenzy_hz=BUZZER_FREQ)
        pyroom.add_source(buzzer, signal=buzzer_signal)
    return pyroom


def main(args=None):
    rclpy.init(args=args)

    sim_node = CrazyflieSimulation()

    rclpy.spin(sim_node)

    sim_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
