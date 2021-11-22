#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
reader_crtp.py: 
"""

import logging
import time

import numpy as np

from cflib.utils.callbacks import Caller
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander
import cflib.crtp

from cflib.crtp.crtpstack import CRTPPort

from crazyflie_description_py.parameters import FFTSIZE, N_MICS

# TODO(FD) for now, below only works with modified cflib.
# CRTP_PORT_AUDIO = CRTPPort.AUDIO
CRTP_PORT_AUDIO = 0x09

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


# Format:
# <name-for-ROS>: (<log name from Crazyflie>, <type>, <scaling>)
#
# see the notes in docs/Motion.md for explanation of parameters.
#
# Note that all parameters are scaled so that they are given in degrees or meters.
# Also note that we change the coordinate systems so that the "front" on Crazyflie
# points in positive y direction, and the x points to the right, looking from aboe.
CHOSEN_LOGGERS = {
    "motion": {
        "yaw_deg": ("stabilizer.yaw", "float"),
        "vx": ("kalman.statePY", "float", -1),
        "vy": ("kalman.statePX", "float"),
        "x": ("kalman.stateY", "float", -1),
        "y": ("kalman.stateX", "float"),
        "z": ("kalman.stateZ", "float"),
    },
    "status": {"vbat": ("pm.vbat", "float"),},
    "motors": {
        "m1_pwm": ("pwm.m1_pwm", "uint32_t"),
        "m2_pwm": ("pwm.m2_pwm", "uint32_t"),
        "m3_pwm": ("pwm.m3_pwm", "uint32_t"),
        "m4_pwm": ("pwm.m4_pwm", "uint32_t"),
        "m1_thrust": ("audio.m1_thrust", "uint16_t"),
        "m2_thrust": ("audio.m2_thrust", "uint16_t"),
        "m3_thrust": ("audio.m3_thrust", "uint16_t"),
        "m4_thrust": ("audio.m4_thrust", "uint16_t"),
    },
}

LOGGING_PERIODS_MS = {
    "motion": 10,
    "status": 1000,
    "motors": 100,
}

CRTP_PAYLOAD = 29  # number of bytes per package

# audio signals data
N_FRAMES_AUDIO = FFTSIZE * N_MICS * 2  # *2 for complex numbers
AUDIO_DTYPE = np.float32
N_FRAMES_FBINS = FFTSIZE
FBINS_DTYPE = np.uint16

# the timestamp is sent in the end of the fbins messages, as an uint32.
N_BYTES_TIMESTAMP = 4
ALLOWED_DELTA_US = 1e6


def test_logging_size(max_size=26):
    sizes = {
        "float": 4,
        "uint32_t": 4,
        "uint16_t": 2,
        "uint8_t": 1,
        "int32_t": 4,
        "int16_t": 2,
        "int8_t": 1,
    }
    for logger, log_dict in CHOSEN_LOGGERS.items():
        size = 0
        for key, vals in log_dict.items():
            size += sizes[vals[1]]
        if size > max_size:
            raise RuntimeError(f"logging config {logger} too big: {size}>{max_size}")


class ArrayCRTP(object):
    def __init__(self, dtype, n_frames, name="array", extra_bytes=0):
        """
        :param n_bytes: the number of bytes to form one array, we will read 
                        CRTP packets until we reach this number of bytes.
        :param dtype: the type of data to be read (np.float32/np.uint16/etc.)
        :param name: name of this array (used for printing only)
        """
        self.name = name
        self.n_frames = n_frames
        self.n_bytes = n_frames * np.dtype(dtype).itemsize + extra_bytes
        self.n_packets_full, self.n_bytes_last = divmod(self.n_bytes, CRTP_PAYLOAD)
        print(f"{name}: waiting for {self.n_bytes} bytes.")
        self.packet_counter = 0
        self.dtype = dtype
        self.packet_start_time = time.time()

        self.bytes_array = np.zeros(self.n_bytes, dtype=np.uint8)
        self.array = np.zeros(n_frames, dtype=dtype)

    def fill_array_from_crtp(self, packet, verbose=False):
        """
        :param packet: CRTP packet
        :param timestamp: current timestamp

        :returns: True if the array was filled, False if it is not full yet.
        """
        if verbose and (self.name == "fbins"):
            print(
                f"ReaderCRTP: filling fbins {self.packet_counter} (every second should be zero or one): {packet.datal}"
            )
        elif verbose and (self.name == "audio"):
            print(
                f"ReaderCRTP: filling audio {self.packet_counter} (first 6 floats): {packet.datal[:6*4]}"
            )

        # received all full packets, read remaining bytes
        if self.packet_counter == self.n_packets_full:
            self.bytes_array[
                self.packet_counter * CRTP_PAYLOAD : self.packet_counter * CRTP_PAYLOAD
                + self.n_bytes_last
            ] = packet.datal[: self.n_bytes_last]

            self.array = np.frombuffer(self.bytes_array, dtype=self.dtype)

            # increase the counter to test for package loss
            self.packet_counter += 1
            return True
        elif self.packet_counter < self.n_packets_full:
            if self.packet_counter == 0:
                self.packet_start_time = time.time()

            assert (self.packet_counter + 1) * CRTP_PAYLOAD < len(
                self.bytes_array
            ), f"{self.name}: index {self.packet_counter * CRTP_PAYLOAD} exceeds length {len(self.array)}"

            self.bytes_array[
                self.packet_counter
                * CRTP_PAYLOAD : (self.packet_counter + 1)
                * CRTP_PAYLOAD
            ] = packet.datal

            self.packet_counter += 1
            return False
        else:
            print(f"unexpected packet: {self.packet_counter} > {self.n_packets_full}")

    def reset_array(self):
        if (self.packet_counter != 0) and (
            self.packet_counter != self.n_packets_full + 1
        ):
            print(
                f"{self.name}: packet loss, received only {self.packet_counter}/{self.n_packets_full+1}."
            )
            success = False
        else:
            success = True

        self.packet_counter = 0
        return success


class ReaderCRTP(object):
    """
    ReaderCRTP recovers the data sent through the CRTP protocol and stores it. 
    There are different schemes for different data types:

    - audio: 
    The audio data (the FFT signals at N_FREQUENCY bins, recorded from N_MICS microphones, and the corresponding frequency bins) is sent in packets of CRTP_PAYLOAD bytes each.
    A new data frame starts when the start condition is met(channel==1) and we count the incoming packets to make sure there is no packet loss. The frequency data is sent after the audio data on channel 2.

    - motion: 
    We read the current motion estimate through the standard logging framework provided by the Crazyflie, and then publish the estimate as a Pose.

    - console: 
    We read whatever is published using DEBUG_PRINT in the firmware and print it out. 

    """

    # TODO(FD) potentially replace with constant read in ROS
    VELOCITY = 0.05  # 3 seconds for 15cm
    BATTERY_OK = 3  # 4.1 #3.83

    def __init__(
        self,
        crazyflie,
        verbose=False,
        log_motion=False,
        log_status=True,
        log_motors=False,
    ):

        test_logging_size()
        self.start_time = time.time()

        self.receivedChar = Caller()
        self.frame_started = False
        self.cf = crazyflie
        self.verbose = verbose

        if self.cf is not None:
            self.mc = MotionCommander(self.cf)
        else:
            self.mc = None

        self.logging_dicts = {
            key: {"timestamp": None, "data": None, "published": True}
            for key in ["motion", "status", "motors"]
        }
        if log_motion:
            self.init_log_config("motion")
        if log_status:
            self.init_log_config("status")
        if log_motors:
            self.init_log_config("motors")

        if self.cf is not None:
            self.cf.add_port_callback(CRTP_PORT_AUDIO, self.callback_audio)
            self.cf.add_port_callback(CRTPPort.CONSOLE, self.callback_console)

        # this data can be read and published by ROS nodes
        self.audio_dict = {
            "timestamp": None,
            "audio_timestamp": None,
            "signals_f_vect": None,
            "fbins": None,
            "published": True,
        }

        self.audio_array = ArrayCRTP(AUDIO_DTYPE, N_FRAMES_AUDIO, "audio")
        self.fbins_array = ArrayCRTP(FBINS_DTYPE, N_FRAMES_FBINS, "fbins")
        self.audio_timestamp = 0

        # for debugging only
        self.update_rate = 0
        self.last_packet_time = 0

        # start sending audio data
        if self.cf is not None:
            self.cf.param.set_value("audio.send_audio_enable", 1)
            if self.verbose:
                print("ReaderCRTP: set audio.send_audio_enable")

    def set_audio_param(self, param):
        if self.cf is not None:
            self.cf.param.set_value(f"audio.{param.name}", param.value)

    def send_disable_motors():
        if self.cf is not None:
            self.cf.param.set_value("motorPowerSet.enable", 0)

    def init_log_config(self, name):
        lg_config = LogConfig(name=name, period_in_ms=LOGGING_PERIODS_MS[name])
        for log_value in CHOSEN_LOGGERS[name].values():
            lg_config.add_variable(log_value[0], log_value[1])

        if self.cf is not None:
            self.cf.log.add_config(lg_config)
            lg_config.data_received_cb.add_callback(self.callback_logging)
            lg_config.start()

    def get_time_ms(self):
        return int((time.time() - self.start_time) * 1000)

    def callback_audio(self, packet):
        # We send the first package this channel to identify the start of new audio data.
        # Channel order:    1 | 0 0 ... 0 0 |  2 2 2
        #         audio start |    audio    |  fbins
        if packet.channel == 1:
            self.frame_started = True
            self.audio_array.reset_array()
            self.fbins_array.reset_array()

        if self.frame_started and (
            packet.channel in [0, 1]
        ):  # channel is either 0 or 1: read audio data
            self.audio_array.fill_array_from_crtp(packet, verbose=False)

        elif self.frame_started and packet.channel == 2:  # channel is 2: read fbins
            filled = self.fbins_array.fill_array_from_crtp(packet, verbose=False)

            if filled:
                # read the timestamp from the last packet
                timestamp_bytes = np.array(
                    packet.datal[
                        self.fbins_array.n_bytes_last : self.fbins_array.n_bytes_last
                        + N_BYTES_TIMESTAMP
                    ],
                    dtype=np.uint8,
                )

                # frombuffer returns array of length 1
                new_audio_timestamp = int(
                    np.frombuffer(timestamp_bytes, dtype=np.uint32)[0]
                )

                # reject faulty packages
                if self.audio_timestamp and (
                    new_audio_timestamp > self.audio_timestamp + ALLOWED_DELTA_US
                ):
                    return
                self.audio_timestamp = new_audio_timestamp

                self.audio_dict["published"] = False
                self.audio_dict["signals_f_vect"] = self.audio_array.array
                self.audio_dict["fbins"] = self.fbins_array.array
                self.audio_dict["audio_timestamp"] = self.audio_timestamp
                self.audio_dict["timestamp"] = self.get_time_ms()

                if self.verbose:
                    packet_time = time.time() - self.audio_dict["timestamp"]
                    print(
                        f"ReaderCRTP callback: time for all packets: {packet_time}s, current timestamp: {new_audio_timestamp}, update rate: {self.update_rate:.2f}"
                    )
                    self.update_rate = 1 / (time.time() - self.last_packet_time)
                    self.last_packet_time = time.time()

    def callback_console(self, packet):
        message = "".join(chr(n) for n in packet.datal)
        print(message, end="")

    def callback_logging(self, timestamp, data, logconf):
        dict_to_fill = self.logging_dicts[logconf.name]
        dict_to_fill["timestamp"] = self.get_time_ms()
        dict_to_fill["published"] = False
        dict_to_fill["data"] = {}
        for key, vals in CHOSEN_LOGGERS[logconf.name].items():
            value = data[vals[0]]
            if len(vals) == 3:
                value /= vals[2]
            dict_to_fill["data"][key] = value
        # if self.verbose:
        #    print(f'ReaderCRTP {logconf.name} callback: {dict_to_fill["data"]}')

    def battery_ok(self):
        battery = self.logging_dicts["status"]["data"]["vbat"]
        if battery is None:
            return True
        elif battery <= self.BATTERY_OK:
            print(f"Warning: battery only at {battery}, not executing command")
            return False
        return True

    def send_thrust_command(self, value, motor="all"):
        if self.cf is None:
            return True

        if (value > 0) and not self.battery_ok():
            return False

        if motor == "all":
            [self.cf.param.set_value(f"motorPowerSet.m{i}", value) for i in range(1, 5)]
        else:
            self.cf.param.set_value(f"motorPowerSet.{motor}", value)

        if value > 0:
            self.cf.param.set_value("motorPowerSet.enable", 1)
        return True

    def send_hover_command(self, height):
        if self.mc is None:
            return True
        if not self.battery_ok():
            return False
        self.mc.take_off(height)
        return True

    def send_turn_command(self, angle_deg):
        if self.mc is None:
            return True
        if angle_deg > 0:
            self.mc.turn_left(angle_deg)
        else:
            self.mc.turn_right(-angle_deg)
        return True
        # do not need this because  it is part of turn_*
        # time.sleep(1)

    def send_move_command(self, distance_m):
        if self.mc is None:
            return True
        if distance_m > 0:
            self.mc.forward(distance_m, velocity=self.VELOCITY)
        else:
            self.mc.back(-distance_m, velocity=self.VELOCITY)
        return True

    def send_forward_command(self, velocity=None):
        if self.mc is None:
            return True
        if velocity is None:
            print("warning: using default", self.VELOCITY)
            self.mc.start_forward(self.VELOCITY)
        elif velocity > 0:
            print("velocity in reader_crtp:", velocity)
            self.mc.start_forward(velocity)
        elif velocity < 0:
            print("velocity in reader_crtp:", velocity)
            self.mc.start_back(-velocity)
        else:
            print("zero-velocity")
            self.mc.stop()
        return True

    def send_land_command(self, velocity=0):
        if self.mc is None:
            return True
        if velocity > 0:
            self.mc.land(velocity)
        else:
            self.mc.land()
        return True

    def send_buzzer_idx(self, idx):
        if self.cf is not None:
            self.cf.param.set_value("audio.buzzer_idx", idx)

    # uses Crazyflie buzzer deck
    def send_buzzer_effect(self, effect):
        if self.cf is not None:
            self.cf.param.set_value("sound.effect", effect)

    # uses Crazyflie buzzer deck
    def send_buzzer_freq(self, freq):
        if self.cf is not None:
            self.cf.param.set_value("sound.freq", freq)

    def send_audio_enable(self, value):
        self.audio_array.reset_array()
        self.fbins_array.reset_array()
        if self.cf is not None:
            self.cf.param.set_value("audio.send_audio_enable", value)


if __name__ == "__main__":
    import argparse

    verbose = True
    log_motors = True
    log_motion = True
    log_status = True
    cflib.crtp.init_drivers(enable_debug_driver=False)

    parser = argparse.ArgumentParser(description="Read CRTP data from Crazyflie.")
    parser.add_argument(
        "id",
        metavar="ID",
        type=int,
        help='number of Crazyflie ("radio://0/[ID]0/2M")',
        default=8,
    )
    args = parser.parse_args()

    id = f"radio://0/{args.id}0/2M/E7E7E7E7E{args.id}"

    with SyncCrazyflie(id) as scf:
        cf = scf.cf

        reader_crtp = ReaderCRTP(
            cf,
            verbose=verbose,
            log_motion=log_motion,
            log_motors=log_motors,
            log_status=log_status,
        )

        try:
            while True:
                time.sleep(1)
        except:
            print("ReaderCRTP: unset audio.send_audio_enable")
            cf.param.set_value("audio.send_audio_enable", 0)
            time.sleep(3)
