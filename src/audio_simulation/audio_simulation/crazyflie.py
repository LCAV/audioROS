"""
crazyflie.py: Publish simulated audio signals for the Crazyflie drone in a room.  

"""

from math import atan2, degrees
import os
import sys
import time

import numpy as np
import pyroomacoustics as pra

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose

from audio_interfaces.msg import Signals, PoseRaw
from audio_interfaces_py.messages import create_signals_message, create_pose_raw_message_from_poses
from audio_simulation.geometry import global_positions, get_relative_movement, get_starting_pose
from audio_simulation.geometry import ROOM_DIM, SOURCE_POS
from crazyflie_description_py.parameters import MIC_POSITIONS, BUZZER_POSITION, FS, N_BUFFER, HEIGHT_MIC_ARRAY, HEIGHT_BUZZER

sys.path.append(os.getcwd() + "/crazyflie-audio/python/")
from signals import generate_signal_random, generate_signal_mono


SOURCE_TYPE = "random" # type of source, random or mono
SOURCE_FREQ = 3500 # Hz, only used for mono.
BUZZER_ON = False # add buzzer to drone.
BUZZER_FREQ = 4000 # Hz, 
MAX_TIMESTAMP = 2**32 - 1  # max value of uint32
NUM_REFLECTIONS = 0 # number of reflections to consider in pyroomacoustis.
DIM = 2 # dimension of simulation
DURATION_SEC = 5 # duration of simulated audio signal 
LOOP = True # flag for looping the signal after reaching the end
NOISE = 1e-2 # white noise to add on signals (variance squared), set to None for no effect


class CrazyflieSimulation(Node):
    def __init__(self):
        super().__init__("audio_simulation")

        self.room = create_room()
        self.time_idx = 0

        self.publisher_pose_raw = self.create_publisher(PoseRaw, "geometry/pose_raw", 10)
        self.publisher_signals = self.create_publisher(Signals, "audio/signals", 10)
        self.subscription_position = self.create_subscription(
            Pose, "geometry/pose", self.pose_listener_callback, 10
        )

        self.create_timer(N_BUFFER/FS, self.signals_publisher)

        self.signals = []
        self.mic_positions_global = []
        self.current_pose = get_starting_pose()
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
            self.end_idx = self.room.fs * DURATION_SEC # this should be less than signals.shape[1]
            # alternative: 
            # nonzero_indices = np.where(np.all(self.signals > 0, axis=0))[0] 
            # self.time_idx = nonzero_indices[0]
            # self.end_idx = nonzero_indices[-1]

            self.get_logger().info(f"Starting again at {self.time_idx}")
            assert self.signals.shape[1] >= self.end_idx
            if not np.any(self.signals[:, self.time_idx]):
                self.get_logger().warn(f"all zero in beginning: {self.signals[0, self.time_idx:self.end_idx]}")
            if not np.any(self.signals[:, self.end_idx]):
                self.get_logger().warn(f"all zero in end: {self.signals[0, self.time_idx:self.end_idx]}")

        timestamp = self.get_time_ms() 
        signal_to_send = self.signals[:, self.time_idx:self.time_idx+N_BUFFER]

        msg = create_signals_message(signal_to_send, self.mic_positions, timestamp, self.room.fs)
        self.publisher_signals.publish(msg)

        if self.pose_raw_msg is not None:
            self.pose_raw_msg.timestamp = timestamp
            self.publisher_pose_raw.publish(self.pose_raw_msg)
            self.pose_raw_msg = None # makes sure not to publish this twice. 
        self.get_logger().info(f"{timestamp}: Published audio and pose signal")

        self.time_idx += N_BUFFER
       
        if self.time_idx >= MAX_TIMESTAMP:
            self.get_logger().warn("timestamp overflow")
        elif (self.time_idx + N_BUFFER) >= self.end_idx:
            self.get_logger().warn("end of signal array")
        else: # continue normally
            return 

        # treat the cases where time_idx has reached limit
        if LOOP:
            self.time_idx = 0 # will be filled next time we enter this function
        else:
            self.get_logger().warn("exiting the node")
            self.destroy_node()
            rclpy.shutdown() # exit the console command


    def get_time_ms(self):
        return int(round(self.time_idx / FS * 1000))


    def pose_listener_callback(self, msg_pose):
        """
        Run a simulation, update the signal content and publish a PoseRaw
        message  when a new pose is received.
        """
        from copy import deepcopy
        t1 = time.time()

        # create a new pose_raw message. 
        if SOURCE_POS is not None:
            source_direction_deg = degrees(atan2(SOURCE_POS[1] - msg_pose.position.y, 
                                                 SOURCE_POS[0] - msg_pose.position.x))
        else:
            source_direction_deg = 0
        self.pose_raw_msg = create_pose_raw_message_from_poses(self.current_pose, msg_pose)
        self.pose_raw_msg.source_direction_deg = source_direction_deg 

        self.mic_positions_global = global_positions(self.mic_positions, msg_pose, z=HEIGHT_MIC_ARRAY)[:, :DIM]
        for m in range(self.mic_positions_global.shape[0]):
            pos = self.mic_positions_global[m]
            if not self.room.is_inside(pos):
                self.get_logger().warn(f"mic{m} position outside room: {pos}")
                return

        # update the signals if this is the first measurement or if we have moved.
        delta = get_relative_movement(self.current_pose, msg_pose)
        if any(delta):
            buzzer = None
            if BUZZER_ON: 
                buzzer = global_positions(self.buzzer_position, msg_pose, z=HEIGHT_BUZZER)

            self.room = self.simulation(self.mic_positions_global, buzzer=buzzer)
             # make a copy to make sure we don't send data while it is being changed.
            self.signals = deepcopy(self.room.mic_array.signals)
        else:
            self.get_logger().info("Did not move, not updating signals.")

        self.current_pose = msg_pose
        self.get_logger().warn(f"Updated signals after {time.time() - t1:.2f}s")
        return


    def simulation(self, mic_array, buzzer=None):
        """
        Run a pyroomacoustics simulation on a copy of the pyroom attribute 
        of an CrazyflieSimulation object.
        """
        t1 = time.time()
        pyroom_copy = create_room()
        #self.get_logger().warn(f"1: {time.time() - t1:.2f}s")
        pyroom_copy.add_microphone_array(mic_array[:, :DIM].T) # dim x n_mics
        #self.get_logger().warn(f"2: {time.time() - t1:.2f}s")
        if buzzer is not None: 
            pyroom_copy.sources[-1].position = buzzer.flatten()[:DIM]
        pyroom_copy.simulate()
        #self.get_logger().warn(f"3: {time.time() - t1:.2f}s")
        return pyroom_copy


def starting_sample_nr(pyroom, mic_array):
    """
    Calculate the index of the sample from which on all mics register 
    audio from all sources.
    """
    # n_sources x dim:
    p_sources = np.array([s.position for s in pyroom.sources]) 
    assert p_sources.shape[1] == mic_array.shape[1]
    # n_sources x n_mics:
    distances = np.linalg.norm(p_sources[:, None, :] - mic_array[None, :, :], axis=2)
    max_distance = np.max(distances)
    max_index = int(np.ceil((max_distance / pyroom.physics.get_sound_speed()) * pyroom.fs) + 1)
    return max_index


def create_room():
    """
    Set the shoe box room with specified dimensions, 
    sampling frequency and sources.
    """
    pyroom = pra.ShoeBox(ROOM_DIM[:DIM], fs=FS, max_order=NUM_REFLECTIONS, sigma2_awgn=NOISE)

    # The max possible delay comes from the main diagonal of cuboid. 
    max_delay_sec = np.linalg.norm(np.array(ROOM_DIM[:DIM])) / pyroom.physics.get_sound_speed()
    assert DURATION_SEC > max_delay_sec

    if SOURCE_POS is not None:
        if SOURCE_TYPE == "random": 
            source_signal = generate_signal_random(FS, DURATION_SEC)
        elif SOURCE_TYPE == "mono":
            source_signal = generate_signal_mono(FS, DURATION_SEC, frequency_hz=SOURCE_FREQ)
        else:
            raise ValueError(SOURCE_TYPE)
        pyroom.add_source(SOURCE_POS[:DIM], signal=source_signal)

    if BUZZER_ON: 
        buzzer = [0, 0, 0] # will be overwritten
        buzzer_signal = generate_signal_mono(FS, DURATION_SEC, frequency_hz=BUZZER_FREQ)
        pyroom.add_source(buzzer[:DIM], signal=buzzer_signal)
    return pyroom


def main(args=None):
    rclpy.init(args=args)

    sim_node = CrazyflieSimulation()

    rclpy.spin(sim_node)

    sim_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
