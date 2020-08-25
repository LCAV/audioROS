import logging
import os
import sys
import time
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir + '/../../../crazyflie-audio/python/')

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion

import matplotlib.pylab as plt
import numpy as np

from audio_interfaces.msg import SignalsFreq, PoseRaw
from audio_stack.live_plotter import LivePlotter

from reader_crtp import ReaderCRTP

logging.basicConfig(level=logging.ERROR)
id = "radio://0/80/2M"

MAX_YLIM = 1e13 # set to inf for no effect.
MIN_YLIM = 1e-13 # set to -inf for no effect.

N_FREQUENCIES = 32
N_MICS = 4
FS = 32000
N = 1024

class AudioPublisher(Node):
    def __init__(self, reader_crtp, plot=False):
        super().__init__('audio_publisher')

        self.publisher_signals = self.create_publisher(SignalsFreq, 'audio/signals_f', 10)
        self.publisher_motion_pose = self.create_publisher(Pose, 'motion/pose', 10)
        self.publisher_motion_pose_raw = self.create_publisher(PoseRaw, 'motion/pose_raw', 10)
        self.plot = plot

        self.prev_position_x = 0
        self.prev_position_y = 0

        if self.plot:
            self.plotter = LivePlotter(MAX_YLIM, MIN_YLIM)
            self.plotter.ax.set_xlabel('angle [rad]')
            self.plotter.ax.set_ylabel('magnitude [-]')

        self.start_time = time.time()

        self.reader_crtp = reader_crtp

        # choose high publish rate so that we introduce as little
        # waiting time as possible
        self.create_timer(0.001, self.publish_current_data)

    def publish_current_data(self):
        if not self.reader_crtp.audio_data['published']:
            self.publish_audio_data()
            self.reader_crtp.audio_data['published'] = True

        if not self.reader_crtp.motion_data['published']:
            self.publish_motion_data()
            self.reader_crtp.motion_data['published'] = True

    def publish_audio_data(self):
        self.get_logger().info(f'Publishing signals.')
        signals_f_vect = self.reader_crtp.audio_data['data']

        # TODO(FD) get this from CRTP
        assert N_FREQUENCIES == len(signals_f_vect) / (N_MICS * 2), f"{N_FREQUENCIES} != {len(signals_f_vect)} / {(N_MICS * 2)}"
        frequencies = np.fft.rfftfreq(n=N, d=1/FS)[:N_FREQUENCIES].astype(np.int)

        signals_f = np.zeros((N_MICS, N_FREQUENCIES), dtype=np.complex128)
        for i in range(N_MICS):
            signals_f[i].real = signals_f_vect[i::N_MICS*2]
            signals_f[i].imag = signals_f_vect[i+N_MICS::N_MICS*2]

        # plot data
        if self.plot:
            labels=[f"mic{i}" for i in range(N_MICS)]
            self.plotter.update_lines(np.abs(signals_f), frequencies, labels=labels)

        # send data
        msg = SignalsFreq()
        msg.frequencies = [int(f) for f in frequencies]
        msg.signals_real_vect = list(signals_f.real.flatten())
        msg.signals_imag_vect = list(signals_f.imag.flatten())
        msg.timestamp = int((time.time()-self.start_time)*1000)
        msg.n_mics = N_MICS
        msg.n_frequencies = N_FREQUENCIES
        self.publisher_signals.publish(msg)

        self.get_logger().info(f'Published audio_data.')

    def publish_motion_data(self):
        motion_dict = self.reader_crtp.motion_data['data']
        # TODO(FD) figure out which one to use.
        #timestamp = int(self.reader_crtp.motion_data['timestamp'])
        timestamp = int((time.time() - self.start_time)*1000)

        msg_pose_raw = PoseRaw()
        msg_pose_raw.dx = motion_dict['dx']
        msg_pose_raw.dy = motion_dict['dy']
        msg_pose_raw.dy = motion_dict['z']
        msg_pose_raw.yaw = motion_dict['yaw']
        msg_pose_raw.timestamp = timestamp 
        self.publisher_motion_pose_raw.publish(msg_pose_raw)

        msg_pose = Pose()
        msg_pose.position = Point() 
        msg_pose.position.x = motion_dict['dx'] + self.prev_position_x
        msg_pose.position.y = motion_dict['dy'] + self.prev_position_y
        msg_pose.position.z = motion_dict['z']
        msg_pose.orientation = Quaternion()

        # TODO(FD): could replace this with tf.transformations or tf2.transformations
        from scipy.spatial.transform import Rotation
        r = Rotation.from_euler('z', motion_dict['yaw'], degrees=True)
        r_quat = r.as_quat() 
        msg_pose.orientation.x = r_quat[0]
        msg_pose.orientation.y = r_quat[1] 
        msg_pose.orientation.z = r_quat[2]
        msg_pose.orientation.w = r_quat[3]
        self.publisher_motion_pose.publish(msg_pose)
        self.get_logger().info('Published motion data.')


def main(args=None):
    from cflib.crazyflie.log import LogConfig
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from cflib.crazyflie.syncLogger import SyncLogger
    import cflib.crtp
    plot = False

    cflib.crtp.init_drivers(enable_debug_driver=False)
    rclpy.init(args=args)

    with SyncCrazyflie(id) as scf:
        cf = scf.cf
        #set_thrust(cf, 43000)
        reader_crtp = ReaderCRTP(cf, verbose=True)
        publisher = AudioPublisher(reader_crtp, plot=plot)
        print('done initializing')

        rclpy.spin(publisher)
        print('done spinning')
        #plt.show()

        while True:
            time.sleep(1)

    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
