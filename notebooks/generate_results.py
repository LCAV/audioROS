#!/usr/bin/env python
# coding: utf-8

# In[ ]:

from copy import deepcopy
import math
import sys

import numpy as np

sys.path.append('crazyflie-audio/python/')

Fs = 32000  # Hz, sampling frequency
duration = 100  # seconds, should be long enough to account for delays and full movement

def generate_signals_pyroom(source, source_signal, mics_rotated, time, noise=0, ax=None):
    import pyroomacoustics as pra
    room = pra.AnechoicRoom(fs=Fs, dim=2)
    room.add_source(source, signal=source_signal, name=f"source")

    beam_former = pra.Beamformer(mics_rotated, Fs)
    room.add_microphone_array(beam_former)
    room.simulate()
    
    time_idx = int(round(time * Fs))
    print(time, Fs, time_idx)
    print('error:', time_idx / Fs, time, (time_idx / Fs) - time)
    
    signals = deepcopy(room.mic_array.signals)
    if noise > 0:
        signals += np.random.normal(scale=noise, size=signals.shape)
    print('shifted signals by', time_idx, signals.shape)
    
    if ax is not None:
        for i in range(signals.shape[0]):
            ax.plot(signals[i])
        ax.axvline(x=time_idx)
    return signals[:, time_idx:]

def generate_signals(source, gt_angle_rad, mics_rotated, frequency_hz, time, noise=0, ax=None):
    from algos_beamforming import get_mic_delays
    from constants import SPEED_OF_SOUND

    n_mics = mics_rotated.shape[0]

    delays_relative = get_mic_delays(mics_rotated, gt_angle_rad)
    delays = np.linalg.norm(mics_rotated[0] - source)/SPEED_OF_SOUND + delays_relative

    times = np.arange(0, duration, step=1/Fs) # n_times
    signals = np.sin(2*np.pi*frequency_hz*(times[None, :] - delays[:, None] - time)) # n_mics x n_times
    signals[times[None, :] < delays[:, None]] = 0.0

    if noise > 0:
        signals += np.random.normal(scale=noise, size=signals.shape)

    if ax is not None:
        for i in range(signals.shape[0]):
            ax.plot(signals[i])
        #ax.set_xlim(0, 100)
    return signals

def from_0_to_2pi(angle):
    angle = (angle + np.pi) % (2 * np.pi) - np.pi # -pi to pi
    if (type(angle) == float) or (angle.ndim == 0):
        angle = angle + 2 * np.pi if angle < 0 else angle
    else:
        angle[angle < 0] += 2 * np.pi
    return angle

def from_0_to_360(angle_deg):
    return 180 / np.pi * from_0_to_2pi(angle_deg / 180 * np.pi)

if __name__ == "__main__":
    from mic_array import get_square_array, get_uniform_array
    from audio_stack.beam_former import rotate_mics
    from audio_stack.beam_former import BeamFormer

    np.random.seed(1)

    ##################### constants
    baseline = 0.108  # meters, square side of mic array
    gt_distance = 10  # meters, distance of source
    gt_angle_deg = 90 # angle of ground truth
    #mics_drone = get_square_array(baseline=baseline, delta=0) # 4 x 2
    mics_drone = get_uniform_array(2, baseline=baseline) # 4 x 2

    ##################### parameters
    n_buffer = 2048
    angular_velocity_deg = 20 # deg/sec, velocity of drone
    time_index = 1000 # idx where signal is non-zero for all positions, found heuristically
    degrees = np.array([0, 20, 45], dtype=np.float) # orientations
    frequency_desired = 600 # Hz

    signal_noise = 1e-3  # noise added to signals
    mics_noise = 1e-3  # noise added to mic positions (rigid)
    degree_noise = 5 # noise added to each degree position, in degrees
    time_quantization = 6 # number of decimal places to keep
    time_noise = 1e-6 # noise added to recording times

    combination_method = "sum"
    normalization_method = "none"

    ##################### generate signals
    mics_drone -= np.mean(mics_drone, axis=0) # center the drone
    gt_angle_rad = gt_angle_deg * np.pi / 180.0
    source = gt_distance * np.array([np.cos(gt_angle_rad), np.sin(gt_angle_rad)])

    frequencies = np.fft.rfftfreq(n_buffer, 1/Fs)

    #TODO(FD) make this modular.
    indices = [np.argmin(np.abs(frequencies - frequency_desired))]

    if angular_velocity_deg != 0:
        times_list = (degrees - degrees[0]) / angular_velocity_deg 
    else:
        times_list = np.zeros(len(degrees))

    assert duration > max(times_list)

    # create noisy versions
    mics_drone_noisy = deepcopy(mics_drone)
    times_list_noisy = deepcopy(times_list)
    degrees_noisy = deepcopy(degrees)
    if mics_noise > 0:
        mics_drone_noisy += np.random.normal(scale=mics_noise, size=mics_drone.shape)
    if time_noise > 0:
        times_list_noisy += np.random.normal(scale=time_noise, size=times_list.shape)
    if time_quantization > 0:
        times_list_noisy = np.round(times_list_noisy, time_quantization)
    if degree_noise > 0:
        degrees_noisy += np.random.normal(scale=degree_noise, size=degrees.shape)

    # we think we move to "degrees" position but we actually move to degrees_noisy.
    mics_list = [rotate_mics(mics_drone_noisy, orientation_deg=degree) for degree in degrees_noisy]
    mics_clean = [rotate_mics(mics_drone, orientation_deg=degree) for degree in degrees]

    frequencies = frequencies[indices]
    frequency_hz = frequencies[0]

    signals_list = []
    for mics, time in zip(mics_list, times_list):
        signals_received =  generate_signals(source, gt_angle_rad, mics, frequency_hz, time, noise=signal_noise) 
        #signals_received = generate_signals_pyroom(source, source_signal, mics.T, time, noise=signal_noise)
        signals_list.append(signals_received)

    buffers = [signals_this[:, time_index:time_index + n_buffer] for signals_this in signals_list] 
    signals_f_list = [np.fft.rfft(buffer).T for buffer in buffers] # n_frequences x 4
    signals_f_list = [sig_f[indices, :] for sig_f in signals_f_list]

    ##################### generate "real" multi-mic signals 

    mics_array = np.concatenate([*mics_list])
    signals_multimic = generate_signals(source, gt_angle_rad, mics_array, frequency_hz, time=0, noise=signal_noise)
    #signals_multimic = generate_signals_pyroom(source, source_signal, mics_array.T, time=0, noise=signal_noise)
    buffer_multimic = signals_multimic[:, time_index:time_index + n_buffer]

    ##################### DOA estimation
    ## multi-mic spectrum
    mics_array_theoretical = np.concatenate([*mics_clean])
    beam_former = BeamFormer(mic_positions=mics_array_theoretical)
    signals_f_multimic = (np.fft.rfft(buffer_multimic).T)[indices, :]
    R_multimic = beam_former.get_correlation(signals_f_multimic)
    spectrum_multimic = beam_former.get_mvdr_spectrum(R_multimic, frequencies)

    ## combined spectra
    beam_former = BeamFormer(mic_positions=mics_drone)
    Rs = [beam_former.get_correlation(sig_f) for sig_f in signals_f_list]
    # TODO(FD) make das or mvdr an option
    #spectra = [beam_former.get_das_spectrum(R, frequencies) for R in Rs]
    spectra = [beam_former.get_mvdr_spectrum(R, frequencies) for R in Rs]
    beam_former.init_dynamic_estimate(combination_n=len(degrees), 
                                      combination_method=combination_method, 
                                      normalization_method=normalization_method)
    for spectrum, degree in zip(spectra, degrees):
        beam_former.add_to_dynamic_estimates(spectrum, -degree)
    combined_spectra = beam_former.get_dynamic_estimate()


    ## combined raw signals
    beam_former = BeamFormer(mic_positions=mics_drone)
    beam_former.init_multi_estimate(frequencies)
    for i, (sig_f, time) in enumerate(zip(signals_f_list, times_list_noisy)):
        beam_former.add_to_multi_estimate(sig_f, frequencies, time, degrees[i])
    # TODO(FD) make das or mvdr an option
    combined_raw = beam_former.get_multi_estimate()
