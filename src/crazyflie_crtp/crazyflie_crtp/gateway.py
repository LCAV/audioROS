import time

import rclpy
from rclpy.node import Node

import matplotlib.pylab as plt
import numpy as np

from audio_interfaces.msg import SignalsFreq
from audio_stack.live_plotter import LivePlotter

MAX_YLIM = 1e13 # set to inf for no effect.
MIN_YLIM = 1e-13 # set to -inf for no effect.

### crayflie stuff
import logging
from cflib.crtp.crtpstack import CRTPPort
from cflib.utils.callbacks import Caller
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
import cflib.crtp

logging.basicConfig(level=logging.ERROR)
id = "radio://0/80/2M"

N_FREQUENCIES = 32
N_MICS = 4
CRTP_PAYLOAD = 29
FLOAT_PRECISION = 4
N_FLOATS = N_FREQUENCIES * N_MICS * 2  # *2 for complex numbers
N_BYTES = N_FLOATS * FLOAT_PRECISION
N_FULL_PACKETS, N_BYTES_LAST_PACKET = divmod(N_BYTES, CRTP_PAYLOAD)

FS = 32000
N = 1024

def set_thrust(cf,thrust):
    thrust_str = f'{thrust}'
    cf.param.set_value('motorPowerSet.m4', thrust_str)
    cf.param.set_value('motorPowerSet.m1', thrust_str)
    cf.param.set_value('motorPowerSet.m2', thrust_str)
    cf.param.set_value('motorPowerSet.m3', thrust_str)
    cf.param.set_value('motorPowerSet.enable', '1')


class AudioPublisher(Node):
    def __init__(self, audio_crtp, plot=False):
        super().__init__('audio_publisher')

        self.publisher_signals = self.create_publisher(SignalsFreq, 'audio/signals_f', 10)
        self.publisher_motion = self.create_publisher(SignalsFreq, 'audio/signals_f', 10)
        self.plot = plot

        if self.plot:
            self.plotter = LivePlotter(MAX_YLIM, MIN_YLIM)
            self.plotter.ax.set_xlabel('angle [rad]')
            self.plotter.ax.set_ylabel('magnitude [-]')

        # Crazyflie stuff
        self.audio_crtp = audio_crtp
        # choose high publish rate so that we introduce as little
        # waiting time as possible
        self.create_timer(0.001, self.publish_current_data)

    def publish_current_data(self):
        if not self.audio_crtp.audio_data['published']:
            self.publish_audio_data()
            self.audio_crtp.audio_data['published'] = True

        if not self.audio_crtp.motion_data['published']:
            self.publish_motion_data()
            self.audio_crtp.motion_data['published'] = True

    def publish_audio_data(self):
        self.get_logger().info(f'Publishing signals.')
        # TODO(FD) get this from CRTP
        signals_f_vect = self.audio_crtp.audio_data['data']

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
        msg.timestamp = int(time.time()*1000)
        msg.n_mics = N_MICS
        msg.n_frequencies = N_FREQUENCIES
        self.publisher_signals.publish(msg)
        self.get_logger().info(f'Published signals.')

def main(args=None):
    plot = False

    cflib.crtp.init_drivers(enable_debug_driver=False)
    rclpy.init(args=args)

    with SyncCrazyflie(id) as scf:
        cf = scf.cf
        #set_thrust(cf, 43000)
        publisher = AudioPublisher(cf, plot=plot)
        print('done initializing')
        plt.show()
        while True:
            time.sleep(1)

    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
