#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
messages.py: ROS-message to and from numpy-array conversions.
"""

import numpy as np

from geometry_msgs.msg import Pose, Point, Quaternion

from audio_interfaces.msg import Signals, SignalsFreq, Correlations, Spectrum, DoaEstimates
from audio_interfaces.msg import PoseRaw


def get_quaternion(yaw_deg):
    from scipy.spatial.transform import Rotation
    r = Rotation.from_euler("z", yaw_deg, degrees=True)
    r_quat = r.as_quat()
    return Quaternion(x=r_quat[0], y=r_quat[1], z=r_quat[2], w=r_quat[3])


def create_pose_message(motion_dict, prev_x, prev_y, timestamp):
    """ Create Pose message. """
    from scipy.spatial.transform import Rotation
    d_local = np.array([motion_dict["dx"], motion_dict["dy"], motion_dict["z"]])

    r = Rotation.from_euler("z", motion_dict["yaw"], degrees=True)
    pos_world = r.apply(d_local) + np.array([prev_x, prev_y, 0])
    
    msg = Pose()
    msg.position = Point(x=pos_world[0], y=pos_world[1], z=pos_world[2])
    msg.orientation = get_quaternion(motion_dict["yaw"])
    return msg


def create_pose_raw_message(motion_dict, timestamp):
    """ Create PoseRaw message. """
    msg = PoseRaw()
    msg.dx = float(motion_dict["dx"])
    msg.dy = float(motion_dict["dy"])
    msg.vx = float(motion_dict.get("vx", 0))
    msg.vy = float(motion_dict.get("vy", 0))
    msg.x = float(motion_dict["x"])
    msg.y = float(motion_dict["y"])
    msg.z = float(motion_dict["z"])
    msg.yaw_deg = float(motion_dict["yaw"])
    msg.yaw_rate_deg = float(motion_dict.get("yaw_rate", 0))
    msg.source_direction_deg = 0.0
    msg.timestamp = timestamp
    return msg


def create_pose_raw_message_from_poses(previous_pose, current_pose):
    """
    Create a PoseRaw message based on previous and current Pose.

    :param previous_pose: previous Pose message.
    :param current_pose: current Pose message.
    :return: PoseRaw message
    """
    from scipy.spatial.transform import Rotation
    r = Rotation.from_quat([
        current_pose.orientation.x,
        current_pose.orientation.y,
        current_pose.orientation.z,
        current_pose.orientation.w
    ])
    yaw_deg = r.as_euler('xyz', degrees=True)[2]

    pose_raw = PoseRaw()
    delta_global = np.array([ 
        current_pose.position.x - previous_pose.position.x,
        current_pose.position.y - previous_pose.position.y,
        current_pose.position.z
    ])
    delta = r.inv().apply(delta_global)
    pose_raw.dx = delta[0]
    pose_raw.dy = delta[1] 
    pose_raw.x = current_pose.position.x
    pose_raw.y = current_pose.position.y
    pose_raw.z = current_pose.position.z
    pose_raw.yaw_deg = yaw_deg

    # have to be filled later: 
    pose_raw.timestamp = 0
    pose_raw.yaw_rate_deg = 0.0
    pose_raw.source_direction_deg = 0.0
    return pose_raw


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


def create_signals_freq_message(signals_f, freqs, mic_positions, timestamp, audio_timestamp, fs):
    """ Create SignalsFreq message. """
    msg = SignalsFreq()

    if mic_positions is not None:
        assert signals_f.shape[1] == mic_positions.shape[0]
        msg.mic_positions = list(mic_positions.flatten().astype(float))
    else:
        msg.mic_positions = []

    if audio_timestamp is None:
        audio_timestamp = timestamp * 1000 # ms to us

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


def read_pose_message(msg):
    """ Read Pose message.  """
    # TODO(FD): could replace this with tf.transformations or tf2.transformations
    from scipy.spatial.transform import Rotation
    new_position = np.array((msg.position.x, msg.position.y))
    quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    r = Rotation.from_quat(quat)
    yaw, pitch, roll = r.as_euler('zyx', degrees=True)
    return new_position, yaw, pitch, roll


def read_pose_raw_message(msg):
    """ Read PoseRaw message.  """
    from scipy.spatial.transform import Rotation
    d_local = np.array((msg.dx, msg.dy, 0))
    v_local = np.array((msg.vx, msg.vy, 0))
    r_world = np.array((msg.x, msg.y, msg.z))
    yaw = msg.yaw_deg
    yaw_rate = msg.yaw_rate_deg
    r = Rotation.from_euler('z', yaw, degrees=True)
    d_world = r.apply(d_local)[:2]
    v_world = r.apply(v_local)[:2]
    return r_world, d_world, v_world, yaw, yaw_rate


def read_signals_message(msg):
    """ Read Signals message.  """
    mic_positions = np.array(msg.mic_positions).reshape((msg.n_mics, -1))
    signals = np.array(msg.signals_vect)
    signals = signals.reshape((msg.n_mics, msg.n_buffer))
    return mic_positions, signals


def read_signals_freq_message(msg):
    """ Read SignalsFreq message.  """
    mic_positions = np.array(msg.mic_positions).reshape((msg.n_mics, -1))
    signals_f = np.array(msg.signals_real_vect) + 1j * np.array(msg.signals_imag_vect)
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
