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
DURATION_SEC = 60 # duration of audio signal simulated
LOOP = False # flag for looping the signal after reaching the end


class CrazyflieSimulation(Node):
    def __init__(self):
        super().__init__("audio_simulation")

        self.room = set_room(ROOM_DIM, [SOURCE_POS,], FS)
        self.time_idx = 0

        self.publisher_pose_raw = self.create_publisher(PoseRaw, "geometry/pose_raw", 10)
        self.publisher_signals = self.create_publisher(Signals, "audio/signals", 10)
        self.subscription_position = self.create_subscription(
            Pose, "geometry/pose", self.pose_listener_callback, 10
        )

        self.create_timer(N_BUFFER/FS, self.constant_publisher)

        self.signals = []
        self.mic_positions_global = []
        self.current_pose = None
        self.mic_positions = np.array(MIC_POSITIONS)
   

    def constant_publisher(self):
        """
        Publish signal chunks at a rate defined by the timer
        """
        if len(self.signals) == 0:
            return 
        
        if self.time_idx == 0:
            self.time_idx = starting_sample_nr(self.room, self.mic_positions_global)

        assert (self.time_idx + N_BUFFER) < self.signals.shape[1]

        signal_to_send = self.signals[:, self.time_idx:self.time_idx+N_BUFFER]
        timestamp = self.get_time_ms() 

        msg = create_signals_message(signal_to_send, self.mic_positions, timestamp, self.room.fs)
        self.publisher_signals.publish(msg)
        self.get_logger().info(f"{timestamp}: Published audio signal")
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
        Run a simulation, update the signal content and publish a PoseRaw() 
        message  when a new pose is received.
        """
        rotation = np.array([
            msg_pose.orientation.x, 
            msg_pose.orientation.y, 
            msg_pose.orientation.z, 
            msg_pose.orientation.w
        ])
        drone_center = np.array([
            msg_pose.position.x, 
            msg_pose.position.y, 
            msg_pose.position.z
        ])

        self.mic_positions_global = global_mic_positions(self.mic_positions, rotation, drone_center)
        timestamp = self.get_time_ms()

        delta = None
        if self.current_pose is not None:
            pose_raw = create_pose_raw_message(self.current_pose, msg_pose, timestamp)
            self.publisher_pose_raw.publish(pose_raw)
            self.get_logger().info(f"{timestamp}: Published PoseRaw message.")

            delta = get_relative_movement(self.current_pose, msg_pose)

        # update the signals if this is the first 
        # measurement or if we have moved.
        if (delta is None) or any(delta):
            sim_room = self.simulation(self.mic_positions_global)
            self.signals = sim_room.mic_array.signals
        elif (delta is not None):
            self.get_logger().info("Did not move, not updating signals.")

        self.current_pose = msg_pose


    def simulation(self, mic_array):
        """
        Run a pyroomacoustics simulation on a copy of the pyroom attribute 
        of an CrazyflieSimulation object.
        """
        pyroom_copy = self.copy_room()
        pyroom_copy.add_microphone_array(mic_array.T) # dim x n_mics
        pyroom_copy.simulate()
        return pyroom_copy


    def copy_room(self):
        """
        Copy the pyroom attribute of an CrazyflieSimulation object.
        """
        pyroom_cp = pra.ShoeBox(self.room.shoebox_dim, fs=self.room.fs)
        for i in range(len(self.room.sources)):
            pyroom_cp.add_source(
                self.room.sources[i].position, signal=self.room.sources[i].signal
            )
        return pyroom_cp


def get_relative_movement(pose1, pose2):
    """ Get the step length (in m) and rotation (in radiants) 
    between pose1 and pose2.

    """
    step_length = np.linalg.norm([
        pose2.position.x - pose1.position.x,
        pose2.position.y - pose1.position.y,
        pose2.position.z - pose1.position.z
    ])
    r1, r2 = [R.from_quat([p.orientation.x,
                           p.orientation.y,
                           p.orientation.z,
                           p.orientation.w]) for p in [pose1, pose2]]
    r = r2 * r1.inv() # "get angle2 - angle1"
    rotation = r.magnitude() # magnitude of rotation, in radiants
    return [step_length, rotation]

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
    Calculate yaw from a quaternion.
    """

    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    yaw = degrees(atan2(2.0 * (y*z + w*x), w*w -  x*x - y*y + z*z))
    
    return yaw


def starting_sample_nr(pyroom, mic_array):
    """
    Calculate the index of the sample from which on all mics register 
    audio from all sources.
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


def set_room(room_dim, source_position_list, fs, duration_sec=DURATION_SEC):
    """
    Set the shoe box room with specified dimensions, 
    sampling frequency and sources.
    """
    pyroom = pra.ShoeBox(room_dim, fs=fs, max_order=NUM_REFLECTIONS)

    # The max possible delay comes from the main diagonal of cuboid. 
    max_delay_sec = np.sqrt(np.sum(np.array(room_dim)**2)) / pyroom.physics.get_sound_speed()
    assert duration_sec > max_delay_sec


    for i in range(len(source_position_list)):
        audio_source = generate_signal_random(fs, duration_sec)
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

    sim_node = CrazyflieSimulation()

    rclpy.spin(sim_node)

    sim_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
