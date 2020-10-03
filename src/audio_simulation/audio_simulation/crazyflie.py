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
import math

from scipy.spatial.transform import Rotation as R
from scipy.io import wavfile

from audio_interfaces.msg import Signals
from geometry_msgs.msg import Pose
from std_msgs.msg import String

N_MICS = 4
MIC_DISTANCE = 0.108                                                                                    # distance between microphones
ROOM_DIM = [2, 2, 2]                                                                                    # room dimensions
SOURCE_POS = [1, 1, 1]                                                                                  # source position
WAV_PATH = os.getcwd() + "/src/audio_simulation/audio_simulation/audio_source/white_noise.wav"          # source .wav file
MAX_TIMESTAMP = pow(2, 32) - 1                                                                          # max value of uint32
MAX_BUFFER = 1024                                                                                       # max value of our buffer

class AudioSimulation(Node):

    def __init__(self):
        super().__init__('audio_simulation')
        
        self.room = set_room(ROOM_DIM, [WAV_PATH,], [SOURCE_POS,])
        self.time_id = 0

        self.publisher_signals = self.create_publisher(Signals, 'audio/signals', 10)
        self.subscription_position = self.create_subscription(Pose, 'geometry/pose', self.listener_callback, 10)

    # run a simulation and send the audio signal in packets when a message is received 
    def listener_callback(self, msg_received):      
        
        rotation_rx = msg_received.orientation                                                 
        drone_center_rx = msg_received.position                                                
        
        rotation = np.array([rotation_rx.x, rotation_rx.y, rotation_rx.z, rotation_rx.w])
        drone_center = np.array([drone_center_rx.x, drone_center_rx.y, drone_center_rx.z])
        
        microphones = generate_mic_position_array(rotation, drone_center)
        sim_room = self.simulation(microphones)         
        signal_to_send = sim_room.mic_array.signals

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
        
            if i == no_packages - 1:
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

    # run a pyroomacoustics simulation on a copy of the pyroom attribute of an AudioSimulation object
    def simulation(self, mic_array):            
        mic_arr = np.c_[mic_array[0], mic_array[1], mic_array[2], mic_array[3],]
        pyroom_copy = self.copy_room()
        pyroom_copy.add_microphone_array(mic_arr)
        pyroom_copy.simulate()
        return pyroom_copy

    # copy the pyroom attribute of an AudioSimulation object
    def copy_room(self):                        
        pyroom_cp = pra.ShoeBox(self.room.shoebox_dim)
        for i in range(len(self.room.sources)):
            pyroom_cp.add_source(self.room.sources[i].position, signal = self.room.sources[i].signal)
        return pyroom_cp


# setting the shoe box room with specified dimensions and sources
def set_room(room_dim, wav_path_list, source_position_list):        
    pyroom = pra.ShoeBox(room_dim)
    for i in range(len(wav_path_list)):
        fs, audio_source = wavfile.read(wav_path_list[i])
        pyroom.add_source(source_position_list[i], signal = audio_source)
    return pyroom


# calculate current mics' postion based on the drone's center coords and rotation
def generate_mic_position_array(rotation, drone_center):        
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

