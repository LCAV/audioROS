#!/usr/bin/env python
# coding: utf-8

from copy import deepcopy
import itertools
import os
import sys

import numpy as np

sys.path.append(
    os.path.dirname(os.path.realpath(__file__)) + "/../crazyflie-audio/python/"
)
from algos_basics import get_mic_delays_near
from signals import generate_signal_mono
from constants import SPEED_OF_SOUND

from audio_stack.beam_former import rotate_mics
from audio_stack.beam_former import BeamFormer
from crazyflie_description_py.parameters import N_BUFFER, FS

DURATION = 7  # seconds, should be long enough to account for delays
COMBINATION_METHOD = "sum"  # can be sum or product
NORMALIZATION_METHOD = "none"  # zero_to_one, zero_to_one_all, sum_to_one
USE_MVDR = True  # use MVDR (otherwise use DAS)

GT_ANGLE_DEG = 90  # angle of ground truth


def generate_signals_pyroom(
    source, mics_rotated, frequency_hz, time, noise=0, ax=None, phase_offset=0
):
    """
    :param mics_rotated: shape n_mics x 2
    """
    import pyroomacoustics as pra

    source_signal = generate_signal_mono(
        FS, DURATION, frequency_hz=frequency_hz, phase_offset=phase_offset
    )

    speed_of_sound = pra.parameters.Physics().get_sound_speed()
    if speed_of_sound != SPEED_OF_SOUND:
        print(
            "discrepancy of speed of sound with pyroomacoustics:",
            speed_of_sound,
            SPEED_OF_SOUND,
        )

    room = pra.AnechoicRoom(fs=FS, dim=2)
    room.add_source(source, signal=source_signal)

    beam_former = pra.Beamformer(mics_rotated.T, FS)
    room.add_microphone_array(beam_former)
    room.simulate()

    start_idx = int(round(time * FS))
    if abs((start_idx / FS) - time) > 1e-3:
        print(
            f"sampling time error: have {start_idx / FS} want {time} error {(start_idx / FS) - time}"
        )

    signals = deepcopy(room.mic_array.signals)
    if noise > 0:
        signals += np.random.normal(scale=noise, size=signals.shape)

    signals = signals[:, start_idx:]

    if ax is not None:
        for i in range(signals.shape[0]):
            ax.plot(signals[i], label=f"mic{i}", color=f"C{i}")
    return signals


def generate_signals_analytical(
    source, mics_rotated, frequency_hz, time, noise=0, ax=None, phase_offset=0
):
    """
    :param mics_rotated: shape n_mics x 2
    """
    delays_relative = get_mic_delays_near(mics_rotated, source)
    delays = np.linalg.norm(mics_rotated[0] - source) / SPEED_OF_SOUND + delays_relative

    times = np.arange(time, time + DURATION, step=1 / FS)  # n_times
    signals = np.sin(
        2 * np.pi * frequency_hz * (times[None, :] - delays[:, None]) + phase_offset
    )  # n_mics x n_times
    # signals[times[None, :] < delays[:, None]] = 0.0

    if noise > 0:
        signals += np.random.normal(scale=noise, size=signals.shape)

    if ax is not None:
        for i in range(signals.shape[0]):
            ax.plot(signals[i], label=f"mic{i}", color=f"C{i}")
    return signals


def from_0_to_2pi(angle):
    angle = (angle + np.pi) % (2 * np.pi) - np.pi  # -pi to pi
    if (type(angle) == float) or (angle.ndim == 0):
        angle = angle + 2 * np.pi if angle <= 0 else angle
    else:
        angle[angle <= 0] += 2 * np.pi
    return angle


def from_0_to_360(angle_deg):
    return 180 / np.pi * from_0_to_2pi(angle_deg / 180 * np.pi)


def inner_loop(
    mics_drone, degrees, times_noisy, signals_f_list, signals_f_multimic, frequencies
):
    mics_clean = [rotate_mics(mics_drone, orientation_deg=degree) for degree in degrees]
    mics_array_theoretical = np.concatenate([*mics_clean])

    ##################### DOA estimation
    ## multi-mic spectrum
    beam_former = BeamFormer(mic_positions=mics_array_theoretical)
    R_multimic = beam_former.get_correlation(signals_f_multimic)
    if USE_MVDR:
        spectrum_multimic = beam_former.get_mvdr_spectrum(R_multimic, frequencies)
    else:
        spectrum_multimic = beam_former.get_das_spectrum(R_multimic, frequencies)

    ## combined spectrum
    beam_former = BeamFormer(mic_positions=mics_drone)
    beam_former.init_dynamic_estimate(
        frequencies,
        combination_n=len(degrees),
        combination_method=COMBINATION_METHOD,
        normalization_method=NORMALIZATION_METHOD,
    )
    for sig_f, degree in zip(signals_f_list, degrees):
        R = beam_former.get_correlation(sig_f)
        if USE_MVDR:
            spectrum = beam_former.get_mvdr_spectrum(R, frequencies)
        else:
            spectrum = beam_former.get_das_spectrum(R, frequencies)
        beam_former.add_to_dynamic_estimates(spectrum, degree)
    spectrum_dynamic = beam_former.get_dynamic_estimate()

    ## combined raw signals
    beam_former = BeamFormer(mic_positions=mics_drone)
    beam_former.init_multi_estimate(frequencies, combination_n=len(degrees))
    for i, (sig_f, time) in enumerate(zip(signals_f_list, times_noisy)):
        beam_former.add_to_multi_estimate(sig_f, frequencies, time, degrees[i])
    Rtot = beam_former.get_correlation(beam_former.signals_f_aligned)
    if USE_MVDR:
        spectrum_delayed = beam_former.get_mvdr_spectrum(
            Rtot, beam_former.frequencies_multi, beam_former.multi_mic_positions
        )
    else:
        spectrum_delayed = beam_former.get_das_spectrum(
            Rtot, beam_former.frequencies_multi, beam_former.multi_mic_positions
        )
    return spectrum_dynamic, spectrum_delayed, spectrum_multimic


def simulate_doa(
    degree_noise_list,
    signal_noise_list,
    time_noise_list,
    frequency_list,
    n_it,
    saveas="",
):
    """
    :param degree_noise_list:  noise added to each degree position, in degrees
    :param signal_noise_list:  noise added to microphone signal
    :param time_noise_list:  noise added to recording times, in seconds
    """
    from mic_array import get_square_array
    from geometry import Context

    df = pd.DataFrame(
        columns=[
            "it",
            "seed",
            "spectrum_multimic",
            "spectrum_dynamic",
            "spectrum_delayed",
            "degree_noise",
            "signal_noise",
            "time_noise",
            "frequency",
        ]
    )

    # fixed quantitites
    frequencies_all = np.fft.rfftfreq(N_BUFFER, 1 / FS)
    mics_noise = 1e-5  # noise added to mic positions (rigid)
    time_quantization = 6  # number of decimal places to keep

    # geometry setup
    gt_distance = 1  # meters, distance of source
    angular_velocity_deg = 20  # deg/sec, velocity of drone
    n_samples = 3
    sampling_time = 0.3  # sampling time

    context = Context.get_crazyflie_setup()
    mics_drone = context.mics
    mics_drone -= np.mean(mics_drone, axis=0)
    # mics_drone = get_uniform_array(2, baseline=baseline) # 4 x 2

    gt_angle_rad = GT_ANGLE_DEG * np.pi / 180.0
    source = gt_distance * np.array([np.cos(gt_angle_rad), np.sin(gt_angle_rad)])
    time_index = int((2 * gt_distance) / SPEED_OF_SOUND * FS)
    times = np.arange(n_samples) * sampling_time
    degrees = times * angular_velocity_deg  # orientations
    assert DURATION > max(times)

    seed = 0
    for degree_noise, time_noise, signal_noise, frequency in itertools.product(
        degree_noise_list, time_noise_list, signal_noise_list, frequency_list
    ):
        print(f"degree noise {degree_noise} of {degree_noise_list}")
        print(f"time noise {time_noise} of {time_noise_list}")
        print(f"signal noise {signal_noise} of {signal_noise_list}")
        print(f"frequency {frequency} of {frequency_list}")

        for it in range(n_it):
            print(f"iteration {it + 1}/{n_it}")
            np.random.seed(seed)
            seed += 1
            ##################### generate signals

            # create noisy versions
            mics_drone_noisy = deepcopy(mics_drone)
            times_noisy = deepcopy(times)
            degrees_noisy = deepcopy(degrees)
            if mics_noise > 0:
                mics_drone_noisy += np.random.normal(
                    scale=mics_noise, size=mics_drone.shape
                )
            if time_noise > 0:
                times_noisy += np.random.normal(scale=time_noise, size=times.shape)
            if time_quantization > 0:
                times_noisy = np.round(times_noisy, time_quantization)
            if degree_noise > 0:
                degrees_noisy += np.random.normal(
                    scale=degree_noise, size=degrees.shape
                )

            # choose frequency bin (for now we make sure that the source signal is one of the available bins)
            f_idx = np.argmin(np.abs(frequencies_all - frequency))
            frequency_hz = frequencies_all[f_idx]
            frequencies = np.array(
                [frequency_hz]
            )  # because algorithms expect multiple frequencies

            ### generate signals at different positions
            signals_f_list = []
            mics_list_noisy = [
                rotate_mics(mics_drone_noisy, orientation_deg=degree)
                for degree in degrees_noisy
            ]
            for mics, time in zip(mics_list_noisy, times_noisy):
                signals_received = generate_signals_analytical(
                    source, mics, frequency_hz, time, noise=signal_noise
                )
                # signals_received = generate_signals_pyroom(source, mics, frequency_hz, time, noise=signal_noise)
                buffer_ = signals_received[:, time_index : time_index + N_BUFFER]
                signals_f = np.fft.rfft(buffer_).T
                signals_f_list.append(signals_f[[f_idx], :])

            ### generate "real" multi-mic signals
            mics_array_noisy = np.concatenate([*mics_list_noisy])
            signals_multimic = generate_signals_analytical(
                source, mics_array_noisy, frequency_hz, time=0, noise=signal_noise
            )
            # signals_multimic = generate_signals_pyroom(source, mics_array.T, frequency_hz, time=0, noise=signal_noise)
            buffer_multimic = signals_multimic[:, time_index : time_index + N_BUFFER]
            signals_f_multimic = (np.fft.rfft(buffer_multimic).T)[[f_idx], :]

            spectrum_dynamic, spectrum_delayed, spectrum_multimic = inner_loop(
                mics_drone,
                degrees,
                times,
                signals_f_list,
                signals_f_multimic,
                frequencies,
            )

            df.loc[len(df), :] = dict(
                it=it,
                seed=seed,
                spectrum_dynamic=spectrum_dynamic,
                spectrum_delayed=spectrum_delayed,
                spectrum_multimic=spectrum_multimic,
                degree_noise=degree_noise,
                time_noise=time_noise,
                frequency=frequency_hz,
            )

        if saveas != "":
            df.to_pickle(saveas)
            print(f"saved intermediate to {saveas}")
    return df


if __name__ == "__main__":
    import pandas as pd

    n_it = 10
    signal_noise_list = [1e-3]  # np.logspace(-5, -1, 5)
    frequency_list = [4000]

    degree_noise_list = np.arange(0, 22, step=2)
    time_noise_list = [0]
    saveas = "results/degree_noise.pkl"
    # simulate_doa(degree_noise_list, signal_noise_list, time_noise_list, frequency_list, n_it, saveas)

    degree_noise_list = [0]
    time_noise_list = np.logspace(-6, -2, 10)
    saveas = "results/time_noise.pkl"
    simulate_doa(
        degree_noise_list,
        signal_noise_list,
        time_noise_list,
        frequency_list,
        n_it,
        saveas,
    )

    # saveas = "results/signal_noise.pkl"
    # simulate_doa(degree_noise_list, signal_noise_list, time_noise_list, frequency_list, n_it, saveas)
