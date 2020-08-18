
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

AUDIO_PORT = 0x09

N_FREQUENCIES = 32
N_MICS = 4
CRTP_PAYLOAD = 29
FLOAT_PRECISION = 4
N_FLOATS = N_FREQUENCIES * N_MICS * 2  # *2 for complex numbers
N_BYTES = N_FLOATS * FLOAT_PRECISION
N_FULL_PACKETS, N_BYTES_LAST_PACKET = divmod(N_BYTES, CRTP_PAYLOAD)

FS = 32000
N = 1024

class AudioPublisher(Node):
    def __init__(self, crazyflie):
        super().__init__('audio_publisher')

        self.publisher_signals = self.create_publisher(SignalsFreq, 'audio/signals_f', 10)
        self.plotter = LivePlotter(MAX_YLIM, MIN_YLIM)
        self.plotter.ax.set_xlabel('angle [rad]')
        self.plotter.ax.set_ylabel('magnitude [-]')

        # Crazyflie stuff
        self.array = np.zeros(N_BYTES, dtype=np.uint8)
        self.receivedChar = Caller()
        self.start = False
        self.index = 0
        self.start_time = 0
        self.cf = crazyflie
        self.cf.add_port_callback(AUDIO_PORT, self.callback_incoming)


    def callback_incoming(self, packet):
        if packet.channel == 1:
            if (self.index != 0) and (self.index != N_FULL_PACKETS + 1):
                print(f"packets loss: received only {self.index}/{N_FULL_PACKETS+1}")
            self.index = 0  # reset index
            self.start = True
            self.start_time = time.time()

        if self.start:
            # received all full packets, read remaining bytes
            if self.index == N_FULL_PACKETS:
                self.array[
                    self.index * CRTP_PAYLOAD : self.index * CRTP_PAYLOAD
                    + N_BYTES_LAST_PACKET
                ] = packet.datal[
                    0:N_BYTES_LAST_PACKET
                ]  # last bytes

                signals_f_vect = np.frombuffer(self.array, dtype=np.float32)
                self.get_logger().info(
                    f"Elapsed time for receiving audio data = {time.time() - self.start_time}s"
                )

                # TODO(FD) get this from CRTP
                frequencies = np.fft.rfftfreq(n=N, d=1/FS)[:N_FREQUENCIES].astype(np.int)
                #frequencies = np.arange(N_FREQUENCIES)


                # signals_f_vect is of structure
                # 
                # [real_1(f1), real_2(f1), real_3(f1), real_4(f1),
                #  imag_1(f1), imag_2(f1), imag_3(f1), imag_4(f1),
                #  ... (f2), ...
                #  ... (fN)]

                signals_f = np.zeros((N_MICS, N_FREQUENCIES), dtype=np.complex128)
                for i in range(N_MICS):
                    signals_f[i].real = signals_f_vect[i::N_MICS*2]
                    signals_f[i].imag = signals_f_vect[i+N_MICS::N_MICS*2]

                # plot data
                labels=[f"mic{i}" for i in range(N_MICS)]
                self.plotter.update_lines(np.abs(signals_f), frequencies, labels=labels)
                self.get_logger().info(f'Published signals.')

                # send data
                msg = SignalsFreq()
                msg.frequencies = [int(f) for f in frequencies]
                msg.signals_real_vect = list(signals_f.real.flatten())
                msg.signals_imag_vect = list(signals_f.imag.flatten())

                msg.timestamp = int(time.time()) # returns integer
                msg.n_mics = N_MICS
                msg.n_frequencies = N_FREQUENCIES
                self.publisher_signals.publish(msg)


            else:
                self.array[
                    self.index * CRTP_PAYLOAD : (self.index + 1) * CRTP_PAYLOAD
                ] = packet.datal

            self.index += 1



def main(args=None):

    cflib.crtp.init_drivers(enable_debug_driver=False)
    rclpy.init(args=args)
    print('ok')

    with SyncCrazyflie(id) as scf:
        cf = scf.cf
        publisher = AudioPublisher(cf)
        print('done initializing')
        plt.show()
        while True:
            time.sleep(1)

    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
