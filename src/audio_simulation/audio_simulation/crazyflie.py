"""
crazyflie.py:
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
MAX_BUFFER = 1000 #TEMPORARY VALUE!!!!                                                                  # max value of float64

class AudioSimulation(Node):

    def __init__(self):
        super().__init__('audio_simulation')
        
        self.room = set_room(ROOM_DIM, WAV_PATH)                                             # creating the room with the audio source
        self.time_id = 0

        self.publisher_signals = self.create_publisher(Signals, 'OK_NOK', 10)
        self.subscription_position = self.create_subscription(Pose, 'crazyflie_position', self.listener_callback, 10)


    def listener_callback(self, msg_received):
        
        rotation_rx = msg_received.orientation                                                  # rotation (quaternion) received
        drone_center_rx = msg_received.position                                                 # drone's center position received
        
        rotation = np.array([rotation_rx.x, rotation_rx.y, rotation_rx.z, rotation_rx.w])       # convertion to numpy array
        drone_center = np.array([drone_center_rx.x, drone_center_rx.y, drone_center_rx.z])
        
        microphones = generate_mic_position_array(rotation, drone_center)
        sim_room = simulation(microphones, self.room)                                           # running the audio simulation        
        signal_to_send = sim_room.mic_array.signals                                             # receiving the signal which is to be sent

        no_packages = math.ceil(len(signal_to_send[0]) / MAX_BUFFER)                            # the number of packages which fit the buffer
        last_pkg_size = len(signal_to_send[0]) % MAX_BUFFER                                     # the size of the last package of data        

        for i in range(no_packages):            
            if self.time_id >= MAX_TIMESTAMP:                                                   # managing timestamp overflow
                self.get_logger().error("timestamp overflow")
                self.time_id = self.time_id % MAX_TIMESTAMP
        
            msg = Signals() 
            msg.timestamp = self.time_id
            msg.fs = self.room.fs
            msg.n_mics = N_MICS
            msg.mic_positions = list(microphones.flatten())
        
            if i == no_packages - 1:                                                            # if it is the last package
                signal = np.zeros((N_MICS, last_pkg_size) dtype = float)
            else:                                                                               # if it is a package of N_BUFFER size
                signal = np.zeros((N_MICS, MAX_BUFFER) dtype = float)
            
            for j in range(N_MICS):                                                             # taking a specific range of data for the package x N_MICS
                signal[j] = signal_to_send[j][i * MAX_BUFFER : (i+1) * MAX_BUFFER]
            
            msg.n_buffer = len(signal[0])
            msg.signals_vect = list(signal.flatten())

            self.publisher_signals.publish(msg)                                                 # publishing a package
            self.get_logger().info('Package nr {i} has been sent')

        
        self.get_logger().info('All packages of the signal nr {self.time_id} have been sent')   # the whole signal has been sent
        self.time_id += 1

        


def set_room(room_dim, wav_path):                                                       # setting the shoe box room
    pyroom = pra.ShoeBox(room_dim)
    fs, audio_source = wavfile.read(wav_path)
    pyroom.add_source(SOURCE_POS, signal = audio_source)
    return pyroom


def simulation(mic_array, pyroom):
    mic_arr = np.c_[mic_array[0], mic_array[1], mic_array[2], mic_array[3],]            # applying contatenation
    pyroom_copy = cp.copy(pyroom)                                                       # creating a copy of the class' room
    pyroom_copy.add_microphone_array(mic_arr)                                           # adding microphones
    pyroom_copy.simulate()
    return pyroom_copy


def generate_mic_position_array(rotation, drone_center):
    rot = R.from_quat(rotation)
    
    mic_lt = drone_center + np.array([-MIC_DISTANCE/2, MIC_DISTANCE/2, 0])              # left top
    mic_rt = drone_center + np.array([MIC_DISTANCE/2, MIC_DISTANCE/2, 0])               # right top
    mic_rb = drone_center + np.array([MIC_DISTANCE/2, -MIC_DISTANCE/2, 0])              # right bottom
    mic_lb = drone_center + np.array([-MIC_DISTANCE/2, -MIC_DISTANCE/2, 0])             # left bottom
    mic_locs = [mic_lt, mic_rt, mic_rb, mic_lb]
     
    # getting mic_new_position = (mic_vector - drone_center_vector) * rotation + drone_center_vector
    
    for i in range(len(mic_locs)):
        mic_locs[i] = rot.apply(mic_locs[i] - drone_center) + drone_center 
    
    return mic_locs



def main(args=None):
    rclpy.init(args=args)

    new_pubsub = AudioSimulation()

    rclpy.spin(new_pubsub)

    # Destroy the node explicitly (otherwise it will be done automatically when the garbage collector destroys the node object)
    new_pubsub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()        

