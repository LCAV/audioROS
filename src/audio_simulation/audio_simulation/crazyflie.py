"""
crazyflie.py: Publish simulated audio signals for a given crazyflie's position
"""

import rclpy
from rclpy.node import Node

import pyroomacoustics as pra 
import numpy as np
import matplotlib.pyplot as plt
import copy as cp
import os
import sys
import math

from scipy.spatial.transform import Rotation as R
from numpy.linalg import norm as distance
from scipy.io import wavfile

from audio_interfaces.msg import Signals
from geometry_msgs.msg import Pose
from std_msgs.msg import String

sys.path.append(os.getcwd() + "/crazyflie-audio/python/")
from signals import generate_signal_random

NUM_SAMPLES = 2048                                                                                      # number of samples which will be send in one go
N_MICS = 4                                                                                              # number of microphones in the room
MIC_DISTANCE = 0.108                                                                                    # distance between microphones
ROOM_DIM = [2, 2, 2]                                                                                    # room dimensions
SOURCE_POS = [1, 1, 1]                                                                                  # source position
MAX_TIMESTAMP = pow(2, 32) - 1                                                                          # max value of uint32
MAX_BUFFER = 1024                                                                                       # max value of our buffer
SPEED_OF_SOUND = 343.0                                                                                  # speed of sound in [m/s]
FS = 8000                                                                                               # sampling frequency [Hz]

class AudioSimulation(Node):

    def __init__(self):
        super().__init__('audio_simulation')
        
        self.room = set_room(ROOM_DIM, NUM_SAMPLES, [SOURCE_POS,], FS)
        self.time_id = 0

        self.publisher_signals = self.create_publisher(Signals, 'audio/signals', 10)
        self.subscription_position = self.create_subscription(Pose, 'geometry/pose', self.listener_callback, 10)

    def listener_callback(self, msg_received):      
        """
        Run a simulation and send the audio signal in packets when a message is received 
        """

        rotation_rx = msg_received.orientation                                                 
        drone_center_rx = msg_received.position                                                
        
        rotation = np.array([rotation_rx.x, rotation_rx.y, rotation_rx.z, rotation_rx.w])
        drone_center = np.array([drone_center_rx.x, drone_center_rx.y, drone_center_rx.z])
        
        microphones = generate_mic_position_array(rotation, drone_center)
        sim_room = self.simulation(microphones)         
        start_sample = starting_sample_nr(sim_room, microphones)
        signals_in_room = sim_room.mic_array.signals
        signal_to_send = np.zeros((N_MICS, NUM_SAMPLES-start_sample), dtype = float)

        for i in range(len(signals_in_room)):
            signal_to_send[i] = signals_in_room[i][start_sample:NUM_SAMPLES]
       
        print(len(signal_to_send[0]))
        no_packages = math.ceil(len(signal_to_send[0]) / MAX_BUFFER)
        last_pkg_size = len(signal_to_send[0]) % MAX_BUFFER        
        
        for i in range(no_packages):            
            if self.time_id >= MAX_TIMESTAMP:
                self.get_logger().error("timestamp overflow")
                self.time_id = self.time_id % MAX_TIMESTAMP
        
            msg = Signals() 
            msg.timestamp = self.time_id
            msg.fs = self.room.fs
            msg.n_mics = N_MICS
            msg.mic_positions = list(microphones.flatten())
        
            if i == no_packages - 1 and last_pkg_size != 0:
                signal = np.zeros((N_MICS, last_pkg_size), dtype = float)
            else:
                signal = np.zeros((N_MICS, MAX_BUFFER), dtype = float)
            
            for j in range(N_MICS):
                signal[j] = signal_to_send[j][i * MAX_BUFFER : (i+1) * MAX_BUFFER]
            
            msg.n_buffer = len(signal[0])
            msg.signals_vect = list(signal.flatten())

            self.publisher_signals.publish(msg)
            self.get_logger().info('Package nr ' + str(i) + ' has been sent')

        
        self.get_logger().info('All packages of the signal nr ' + str(self.time_id) + ' have been sent')
        self.time_id += 1

    def simulation(self, mic_array):            
        """
        Run a pyroomacoustics simulation on a copy of the pyroom attribute of an AudioSimulation object
        """
        
        mic_arr = np.c_[mic_array[0], mic_array[1], mic_array[2], mic_array[3],]
        pyroom_copy = self.copy_room()
        pyroom_copy.add_microphone_array(mic_arr)
        pyroom_copy.simulate()
        return pyroom_copy

    def copy_room(self):                        
        """
        Copy the pyroom attribute of an AudioSimulation object
        """
        
        pyroom_cp = pra.ShoeBox(self.room.shoebox_dim)
        for i in range(len(self.room.sources)):
            pyroom_cp.add_source(self.room.sources[i].position, signal = self.room.sources[i].signal)
        return pyroom_cp


def starting_sample_nr(pyroom, mic_array):
    """
    Calculate the number of sample, from which on all mics signals register audio from sources
    """
    
    n_sources = pyroom.n_sources
    n_mics = len(mic_array)
    dist_array = np.empty([n_mics, 1])
    tmp_dist = np.empty([n_sources, 1])

    for i in range(n_mics):
        for j in range(n_sources):
            tmp_dist[j] = distance(pyroom.sources[j].position - mic_array[i])
        dist_array[i] = np.max(tmp_dist)
    
    max_distance = np.max(dist_array)
    max_sample = int(np.ceil((max_distance/SPEED_OF_SOUND) * pyroom.fs) + 1)
    return max_sample

def set_room(room_dim, nr_samples, source_position_list, fs):        
    """
    Set the shoe box room with specified dimensions, sampling frequency and sources
    """
    
    pyroom = pra.ShoeBox(room_dim, fs=fs)
    audio_source = generate_signal_random(nr_samples)
    for i in range(len(source_position_list)):
        pyroom.add_source(source_position_list[i], signal = audio_source)
    return pyroom


def generate_mic_position_array(rotation, drone_center):        
    """
    Calculate current mics' postion based on the drone's center coords and rotation
    """
    
    rot = R.from_quat(rotation)
    
    mic_lt = drone_center + np.array([-MIC_DISTANCE/2, MIC_DISTANCE/2, 0])              # left top
    mic_rt = drone_center + np.array([MIC_DISTANCE/2, MIC_DISTANCE/2, 0])               # right top
    mic_rb = drone_center + np.array([MIC_DISTANCE/2, -MIC_DISTANCE/2, 0])              # right bottom
    mic_lb = drone_center + np.array([-MIC_DISTANCE/2, -MIC_DISTANCE/2, 0])             # left bottom
    mic_locs = [mic_lt, mic_rt, mic_rb, mic_lb]
    
    for i in range(len(mic_locs)):
        mic_locs[i] = rot.apply(mic_locs[i] - drone_center) + drone_center 
   
    mics = np.array(mic_locs)

    return mics



def main(args=None):
    rclpy.init(args=args)

    new_pubsub = AudioSimulation()

    rclpy.spin(new_pubsub)

    new_pubsub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()        

