"""
simulation.py:
"""

import rclpy
from rclpy.node import Node

import pyroomacoustics as pra 
import numpy as np
import matplotlib.pyplot as plt
import os

from scipy.spatial.transform import Rotation as R
from scipy.io import wavfile
from scipy.signal import decimate 
from operator import add
from operator import neg

from audio_interfaces.msg import Signals
from geometry_msgs.msg import Pose
from std_msgs.msg import String

N_MICS = 4
MIC_DISTANCE = 0.108                                                                                    # distance between microphones
ROOM_DIM = [2, 2, 2]                                                                                    # room dimensions
SOURCE_POS = [1, 1, 1]                                                                                  # source position
WAV_PATH = os.getcwd() + "/src/audio_simulation/audio_simulation/audio_source/white_noise.wav"          # source .wav file

class AudioSimulation(Node):

    def __init__(self):
        super().__init__('audio_simulation')
        
#        self.room = set_room(ROOM_DIM, WAV_PATH)                                             # creating the room with the audio source

        self.publisher_signals = self.create_publisher(Signals, 'OK_NOK', 10)
        self.subscription_position = self.create_subscription(Pose, 'crazyflie_position', self.listener_callback, 10)

    def listener_callback(self, msg_received):
        
        rotation_rx = msg_received.orientation                                          # rotation (quaternion) received
        drone_center_rx = msg_received.position                                         # drone's center position received
        
        rotation = [rotation_rx.x, rotation_rx.y, rotation_rx.z, rotation_rx.w]         # convertion to list
        drone_center = [drone_center_rx.x, drone_center_rx.y, drone_center_rx.z]
        
        room = set_room(ROOM_DIM, WAV_PATH)
        microphones_sim, microphones_msg = generate_mic_position_array(rotation, drone_center)
        simulation(microphones_sim, room)                                                       # running the audio simulation

        msg = Signals()
        msg.timestamp = 0
        msg.fs = room.fs
        msg.n_mics = N_MICS
        
        signal = [0, 0, 0, 0]

        for i in range(len(room.mic_array.signals)):                                # decimating signal
            signal[i] = decimate(room.mic_array.signals[i], 8)
        

        msg.n_buffer = len(signal[0])
        
        signal = np.array(signal, dtype = float)                                    # converting signals to np.ndarray
        microphones_msg = np.array(microphones_msg, dtype = float)

        msg.signals_vect = list(signal.flatten())
        msg.mic_positions= list(microphones_msg.flatten())

        self.publisher_signals.publish(msg)
        self.get_logger().info('Data was sent')


def set_room(room_dim, wav_path):                                                   # setting the shoe box room
    pyroom = pra.ShoeBox(room_dim)
    fs, audio_source = wavfile.read(wav_path)
    pyroom.add_source(SOURCE_POS, signal = audio_source)
    return pyroom


def simulation(mic_array, pyroom):
    pyroom.add_microphone_array(mic_array)                                          # adding microphones
    pyroom.simulate()


def generate_mic_position_array(rotation, drone_center):
    rot = R.from_quat(rotation)
    drone_center_rev = list(map(neg, drone_center))
    mic_lt = list( map(add, drone_center, [-MIC_DISTANCE/2, MIC_DISTANCE/2, 0]))    # left top
    mic_rt = list( map(add, drone_center, [MIC_DISTANCE/2, MIC_DISTANCE/2, 0]))     # right top
    mic_rb = list( map(add, drone_center, [MIC_DISTANCE/2, -MIC_DISTANCE/2, 0]))    # right bottom
    mic_lb = list( map(add, drone_center, [-MIC_DISTANCE/2, -MIC_DISTANCE/2, 0]))   # left bottom
    mic_locs = [mic_lt, mic_rt, mic_rb, mic_lb]
     
    # getting mic_new_position = (mic_vector - drone_center_vector) * rotation + drone_center_vector
    
    for i in range(len(mic_locs)):
        tmp = list( map( add, mic_locs[i], drone_center_rev ) )                     # tmp = (mic_vector - drone_center_vector)
        mic_locs[i] = list( map(add, rot.apply(tmp), drone_center))                 # (tmp * rotation) + drone_center_vector

    mic_arr = np.c_[mic_locs[0], mic_locs[1], mic_locs[2], mic_locs[3],]            
    
    return mic_arr, mic_locs                                                        # mic_arr format is needed for simulation, mic_locs for sending the message


def main(args=None):
    rclpy.init(args=args)

    new_pubsub = AudioSimulation()

    rclpy.spin(new_pubsub)

    # Destroy the node explicitly (otherwise it will be done automatically when the garbage collector destroys the node object)
    new_pubsub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()        

