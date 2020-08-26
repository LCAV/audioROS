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
        
        room = SetRoom(ROOM_DIM, WAV_PATH)                                              # creating the room with the audio source

        self.publisher_signals = self.create_publisher(String, 'OK_NOK', 10)
        self.subscription_position = self.create_subscription(Pose, 'crazyflie_position', self.listener_callback, 10)

    def listener_callback(self, msg_received):
        
        rotation_rx = msg_received.orientation                                          # rotation (quaternion) received
        drone_center_rx = msg_received.position                                         # drone's center position received
        
        rotation = [rotation_rx.x, rotation_rx.y, rotation_rx.z, rotation_rx.w]         # convertion to list
        drone_center = [drone_center_rx.x, drone_center_rx.y, drone_center_rx.z]

        # plot the room
        #room.plot(img_order=0)
        #plt.show()

        #msg = String()
        #msg.data = 'Something was sent'
        #self.publisher_signals.publish(msg)
        #self.get_logger().info('Received data')


def SetRoom(room_dim, wav_path):
    pyroom = pra.ShoeBox(room_dim)
    fs, audio_source = wavfile.read(wav_path)
    pyroom.add_source(SOURCE_POS, signal = audio_source)
    return pyroom

#def Simulation(rotation, pose, pyroom):
    #rot = R.from_quat(ROTATION)
    #DRONE_CENTER_REV = list(map(neg, DRONE_CENTER))
    #MIC_LT = list( map(add, DRONE_CENTER, [-MIC_DISTANCE/2, MIC_DISTANCE/2, 0]))    # left top
    #MIC_RT = list( map(add, DRONE_CENTER, [MIC_DISTANCE/2, MIC_DISTANCE/2, 0]))     # right top
    #MIC_RB = list( map(add, DRONE_CENTER, [MIC_DISTANCE/2, -MIC_DISTANCE/2, 0]))    # right bottom
    #MIC_LB = list( map(add, DRONE_CENTER, [-MIC_DISTANCE/2, -MIC_DISTANCE/2, 0]))   # left bottom
    #mic_locs = [MIC_LT, MIC_RT, MIC_RB, MIC_LB]
     
    # MIC_new_position = (MIC_vector - DRONE_CENTER_vector) * rotation + DRONE_CENTER_vector
    #for i in range(len(mic_locs)):
    #    tmp = list( map( add, mic_locs[i], DRONE_CENTER_REV ) )         # tmp = (MIC_vector - DRONE_CENTER_vector)
    #    mic_locs[i] = list( map(add, rot.apply(tmp), DRONE_CENTER))     # (tmp * rotation) + DRONE_CENTER_vector

    #mic_arr = np.c_[    mic_locs[0], mic_locs[1], mic_locs[2], mic_locs[3],    ]

    #room.add_microphone_array(mic_arr)
    #room.simulate()

def main(args=None):
    rclpy.init(args=args)

    new_pubsub = AudioSimulation()

    rclpy.spin(new_pubsub)

    # Destroy the node explicitly (otherwise it will be done automatically when the garbage collector destroys the node object)
    new_pubsub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()        

