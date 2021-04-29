#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
messages.py: ROS-message to and from numpy-array conversions.
"""

import numpy as np

from geometry_msgs.msg import PoseStamped, Point, Quaternion

from audio_interfaces.msg import (
    Signals,
    SignalsFreq,
    Correlations,
    Spectrum,
    DoaEstimates,
    GroundTruth,
)
from audio_interfaces.msg import PoseRaw

from builtin_interfaces.msg import Time


def get_quaternion(yaw_deg, pitch_deg=0, roll_deg=0):
    from scipy.spatial.transform import Rotation

    r = Rotation.from_euler("xyz", [pitch_deg, roll_deg, yaw_deg], degrees=True)
    r_quat = r.as_quat()
    return Quaternion(x=r_quat[0], y=r_quat[1], z=r_quat[2], w=r_quat[3])


def fill_header(msg, timestamp):
    sec, nanosec = convert_ms_to_sec_nanosec(timestamp)
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = nanosec
    msg.header.frame_id = "global"


def convert_ms_to_sec_nanosec(timestamp_ms):
    sec = int(timestamp_ms // 1000)
    nanosec = int(round(timestamp_ms * 1e6 - sec * 1e9))
    return sec, nanosec


def convert_sec_nanosec_to_ms(sec, nanosec):
    return int(round(sec * 1e-3 + nanosec * 1e-6))


def create_pose_message(x, y, z, yaw_deg, timestamp=None, **dump):
    """ Create PoseStamped message. """
    msg = PoseStamped()

    if timestamp is not None:
        fill_header(msg, timestamp)

    msg.pose.position = Point(x=x, y=y, z=z)
    msg.pose.orientation = get_quaternion(yaw_deg)
    return msg


def create_pose_message_from_arrays(quat, position, timestamp=None):
    """ Create PoseStamped message. """
    msg = PoseStamped()

    if timestamp is not None:
        fill_header(msg, timestamp)

    msg.pose.position = Point(x=position[0], y=position[1], z=position[2])
    msg.pose.orientation = Quaternion(
        x=quat[0], y=quat[1], z=quat[2], w=quat[3]
    )
    return msg


def create_pose_raw_message(
    x, y, z, yaw_deg, timestamp, yaw_rate_deg=0, vx=0, vy=0, **dump
):
    """ Create PoseRaw message. """
    msg = PoseRaw(
        vx=float(vx),
        vy=float(vy),
        x=float(x),
        y=float(y),
        z=float(z),
        yaw_deg=float(yaw_deg),
        yaw_rate_deg=float(yaw_rate_deg),
        timestamp=timestamp,
    )
    return msg


def create_signals_message(signals, mic_positions, timestamp, fs):
    """ Create Signals message. """
    msg = Signals()
    msg.timestamp = timestamp
    msg.fs = fs
    msg.n_mics = signals.shape[0]
    msg.n_buffer = signals.shape[1]
    # this is very unlikely to happen and is
    # probably due to signals having the wrong shape.
    assert msg.n_mics < msg.n_buffer, f"invalid signals shape {signals.shape}"
    msg.signals_vect = list(signals.flatten().astype(float))
    if mic_positions is not None:
        msg.mic_positions = list(mic_positions.flatten().astype(float))
    else:
        msg.mic_positions = []
    return msg


def create_signals_freq_message(
    signals_f, freqs, mic_positions, timestamp, audio_timestamp, fs
):
    """ Create SignalsFreq message. """
    msg = SignalsFreq()

    if mic_positions is not None:
        assert signals_f.shape[1] == mic_positions.shape[0]
        msg.mic_positions = list(mic_positions.flatten().astype(float))
    else:
        msg.mic_positions = []

    if audio_timestamp is None:
        audio_timestamp = timestamp * 1000  # ms to us

    msg.fs = fs
    msg.timestamp = timestamp
    msg.audio_timestamp = audio_timestamp
    msg.n_mics = signals_f.shape[1]
    msg.n_frequencies = len(freqs)
    msg.frequencies = [int(f) for f in freqs]

    assert signals_f.shape[0] == msg.n_frequencies
    assert signals_f.shape[1] == msg.n_mics
    # important: signals_f should be of shape n_mics x n_frequencies before flatten() is called.
    msg.signals_real_vect = list(np.real(signals_f.T).astype(float).flatten())
    msg.signals_imag_vect = list(np.imag(signals_f.T).astype(float).flatten())
    return msg


def create_correlations_message(R, freqs, mic_positions, timestamp):
    """ Create Correlations message.

    :param R: autocorrelation matrix (n_freqs, n_mics, n_mics)
    :param freqs: list of frequencies in Hz
    :param mic_positions: mic coordinates (n_mics, 2)
    :param timestamp: timestamp (uint32)

    """
    msg = Correlations()
    msg.n_mics = int(R.shape[1])
    msg.n_frequencies = len(freqs)
    msg.frequencies = [int(f) for f in freqs]
    msg.corr_real_vect = list(R.real.astype(float).flatten())
    msg.corr_imag_vect = list(R.imag.astype(float).flatten())
    msg.mic_positions = list(mic_positions.astype(float).flatten())
    msg.timestamp = timestamp
    return msg


def create_spectrum_message(spectrum, frequencies, timestamp):
    """ Create Spectrum message. """
    msg = Spectrum()
    msg.timestamp = timestamp
    msg.n_frequencies = len(frequencies)
    msg.n_angles = spectrum.shape[1]
    msg.frequencies = list(frequencies.astype(float))
    msg.spectrum_vect = list(spectrum.astype(float).flatten())
    return msg


def create_doa_message(doa_estimates, timestamp):
    """ Create DoaEstimates message. """
    msg = DoaEstimates()
    msg.n_estimates = len(doa_estimates)
    msg.timestamp = timestamp
    msg.doa_estimates_deg = list(doa_estimates.astype(float).flatten())
    return msg


def create_ground_truth_message(
    source_direction_deg=None,
    approach_angle_deg=None,
    wall_distance_m=None,
    wall_angle_deg=None,
    timestamp=None,
    **dump,
):
    """ Create ground truth message. """
    kwargs = {
        k: val
        for k, val in dict(
            timestamp=timestamp,
            source_direction_deg=source_direction_deg,
            approach_angle_deg=approach_angle_deg,
            wall_distance_m=wall_distance_m,
            wall_angle_deg=wall_angle_deg,
        ).items()
        if val is not None
    }
    msg = GroundTruth(**kwargs)
    return msg


def read_pose_message(msg):
    """ Read Pose message.  """
    from scipy.spatial.transform import Rotation

    new_position = np.array((msg.pose.position.x, msg.pose.position.y))
    quat = [
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w,
    ]
    r = Rotation.from_quat(quat)
    yaw, pitch, roll = r.as_euler("zyx", degrees=True)
    return new_position, yaw, pitch, roll


def read_pose_raw_message(msg):
    """ Read PoseRaw message.  """
    from scipy.spatial.transform import Rotation

    v_local = np.array((msg.vx, msg.vy, 0))
    r_world = np.array((msg.x, msg.y, msg.z))
    yaw = msg.yaw_deg
    yaw_rate = msg.yaw_rate_deg
    r = Rotation.from_euler("z", yaw, degrees=True)
    v_world = r.apply(v_local)[:2]
    return r_world, v_world, yaw, yaw_rate


def read_signals_message(msg):
    """ Read Signals message.  """
    mic_positions = np.array(msg.mic_positions).reshape((msg.n_mics, -1))
    signals = np.array(msg.signals_vect)
    signals = signals.reshape((msg.n_mics, msg.n_buffer))
    return mic_positions, signals


def read_signals_freq_message(msg):
    """ Read SignalsFreq message.  """
    mic_positions = np.array(msg.mic_positions).reshape((msg.n_mics, -1))
    signals_f = np.array(msg.signals_real_vect) + 1j * np.array(
        msg.signals_imag_vect
    )
    signals_f = signals_f.reshape((msg.n_mics, msg.n_frequencies)).T
    freqs = np.array(msg.frequencies)
    return mic_positions, signals_f, freqs


def read_correlations_message(msg):
    """ Read Correlations message. """
    if msg.mic_positions:
        mic_positions = np.array(msg.mic_positions).reshape((msg.n_mics, -1))
    else:
        mic_positions = None
    frequencies = np.array(msg.frequencies).astype(np.float)  # [10, 100, 1000]
    R = np.array(msg.corr_real_vect) + 1j * np.array(msg.corr_imag_vect)
    R = R.reshape((len(frequencies), msg.n_mics, msg.n_mics))
    return mic_positions, R, frequencies


def read_spectrum_message(msg):
    """ Read Spectrum message. """
    spectrum = np.array(msg.spectrum_vect).reshape(
        (msg.n_frequencies, msg.n_angles)
    )
    frequencies = np.array(msg.frequencies)
    theta_scan = np.linspace(0, 360, msg.n_angles)
    return spectrum, frequencies, theta_scan
