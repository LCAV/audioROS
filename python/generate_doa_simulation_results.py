#!/usr/bin/env python
# coding: utf-8

from copy import deepcopy
import itertools
import os
import sys

import numpy as np

from utils.algos_basics import get_mic_delays_near, get_mic_delays
from utils.constants import SPEED_OF_SOUND
from utils.signals import generate_signal_mono

from audio_stack.beam_former import rotate_mics
from audio_stack.beam_former import BeamFormer
from crazyflie_description_py.parameters import N_BUFFER, FS

DURATION = 7  # seconds, should be long enough to account for delays
COMBINATION_METHOD = "sum"  # can be sum or product
NORMALIZATION_METHOD = "none"  # zero_to_one, zero_to_one_all, sum_to_one
METHOD = "mvdr"

GT_DISTANCE = 1  # meters, distance of source
SAMPLING_TIME = 0.5  # sampling time in sconds


def generate_signals_pyroom(
    source, mics_rotated, frequency_hz, time, noise=0, ax=None, phase_offset=0
):
    """
    :param mics_rotated: shape n_mics x 2
    """
    import pyroomacoustics as pra

    # add multiple sources by superposition
    if type(source) == list:
        signals = None
        for s, p in zip(source, phase_offset):
            new_signal = generate_signals_pyroom(
                s, mics_rotated, frequency_hz, time, noise, ax, p, farfield
            )
            if signals is None:
                signals = new_signal
            else:
                signals += new_signal
        return signals

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
    source,
    mics_rotated,
    frequency_hz,
    time,
    noise=0,
    ax=None,
    phase_offset=0,
    farfield=False,
):
    """
    :param mics_rotated: shape n_mics x 2
    """
    # add multiple sources by superposition
    if type(source) == list:
        signals = None
        for s, p in zip(source, phase_offset):
            new_signal = generate_signals_analytical(
                s, mics_rotated, frequency_hz, time, noise, ax, p, farfield
            )
            if signals is None:
                signals = new_signal
            else:
                signals += new_signal
        return signals

    if farfield:
        azimuth = np.arctan2(source[1], source[0])
        delays_relative = get_mic_delays(mics_rotated, azimuth=azimuth)
    else:
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


def move_mics(mics_local, degrees, offsets, noise_dict={}):
    mics_list = []
    for degree, offset in zip(degrees, offsets):
        degree_noise = noise_dict.get("degree", 0)
        offset_noise = noise_dict.get("offset", 0)
        if degree_noise > 0:
            degree += np.random.normal(scale=degree_noise)
        if offset_noise > 0:
            assert offset_noise < 1, "make sure offset noise is in m and below 1"
            offset += np.random.normal(scale=offset_noise)
        # import pudb; pudb.set_trace()
        mics_list.append(
            rotate_mics(mics_local, orientation_deg=degree)
            + np.array([offset, 0])[None, :]
        )
    return mics_list


def get_source(angle_rad, source_distance=1, degrees=False):
    if np.ndim(angle_rad) == 0:
        if degrees:
            angle_rad = angle_rad / 180 * np.pi
        return source_distance * np.array((np.cos(angle_rad), np.sin(angle_rad)))
    else:
        if degrees:
            angle_rad = deepcopy(angle_rad)
            angle_rad = np.array(angle_rad) / 180 * np.pi
        return [get_source(a) for a in angle_rad]


def get_movement(angular_velocity_deg, linear_velocity_cm, n_samples, sampling_time):
    times_list = sampling_time * np.arange(n_samples)
    assert DURATION > max(times_list)

    degrees = times_list * angular_velocity_deg
    offsets = times_list * linear_velocity_cm * 1e-2
    return degrees, offsets, times_list


def get_noisy_times(time_list, time_noise, time_quantization=6):
    time_list_noisy = deepcopy(time_list)
    time_list_noisy = time_list + np.random.normal(
        scale=time_noise, size=time_list.shape
    )
    if time_quantization > 0:
        time_list_noisy = np.round(time_list_noisy, time_quantization)
    return time_list_noisy


def inner_loop(
    mics_local,
    degrees,
    offsets,
    times_noisy,
    signals_f_list,
    signals_f_multimic,
    frequencies,
):
    mics_clean = move_mics(mics_local, degrees, offsets)
    mics_array_clean = np.concatenate([*mics_clean])

    ##################### DOA estimation
    ## multi-mic spectrum
    beam_former = BeamFormer(mic_positions=mics_array_clean)
    R_multimic = beam_former.get_correlation(signals_f_multimic)
    if METHOD == "mvdr":
        spectrum_multimic = beam_former.get_mvdr_spectrum(R_multimic, frequencies)
    else:
        spectrum_multimic = beam_former.get_das_spectrum(R_multimic, frequencies)

    ## combined spectrum
    beam_former = BeamFormer(mic_positions=mics_local)
    beam_former.init_multi_estimate(frequencies, combination_n=len(degrees))
    beam_former.init_dynamic_estimate(
        frequencies,
        combination_n=len(degrees),
        combination_method=COMBINATION_METHOD,
        normalization_method=NORMALIZATION_METHOD,
    )
    for sig_f, time, degree, offset in zip(
        signals_f_list, times_noisy, degrees, offsets
    ):
        beam_former.add_signals_to_dynamic_estimates(
            sig_f, frequencies, degree, method=METHOD
        )
        beam_former.add_to_multi_estimate(sig_f, frequencies, time, degree, offset)

    spectrum_dynamic = beam_former.get_dynamic_estimate()
    spectrum_delayed = beam_former.get_multi_estimate(method=METHOD)
    return spectrum_dynamic, spectrum_delayed, spectrum_multimic


def simulate_doa(
    gt_angle_deg,
    degree_noise_list,
    offset_noise_list,
    signal_noise_list,
    time_noise_list,
    frequency_list,
    n_samples,
    n_it,
    linear_velocity_cm_list=[0],
    angular_velocity_deg_list=[0],  # deg/sec, velocity of drone
    saveas="",
    verbose=False,
):
    """
    :param degree_noise_list:  noise added to each degree position, in degrees
    :param signal_noise_list:  noise added to microphone signal
    :param time_noise_list:  noise added to recording times, in seconds
    :param n_samples: number of positions to consider
    :param linear_velocity_cm_list:  linear velocity of drone in cm/s
    :param angular_velocity_deg_list:  velocity of drone in deg/sec
    """
    import progressbar
    from utils.geometry import Context

    df = pd.DataFrame(
        columns=[
            "it",
            "seed",
            "degree_noise",
            "offset_noise" "signal_noise",
            "time_noise",
            "frequency",
            "angular_velocity_deg",
            "linear_velocity_cm",
            "spectrum_multimic",
            "spectrum_dynamic",
            "spectrum_delayed",
        ]
    )

    # fixed quantitites
    frequencies_all = np.fft.rfftfreq(N_BUFFER, 1 / FS)
    time_quantization = 6  # number of decimal places to keep

    # geometry setup
    gt_distance = GT_DISTANCE  # meters, distance of source
    sampling_time = SAMPLING_TIME  # sampling time in sconds

    context = Context.get_crazyflie_setup()
    mics_local = context.mics
    mics_local -= np.mean(mics_local, axis=0)

    # can have one more multiple sources.
    sources = get_source(gt_angle_deg, degrees=True, source_distance=gt_distance)
    if type(gt_angle_deg) == list:
        phase_offsets = np.random.uniform(0, 2 * np.pi, size=len(gt_angle_deg))
    else:
        phase_offsets = 0

    time_index = int((2 * gt_distance) / SPEED_OF_SOUND * FS)

    seed = 0

    combinations = list(
        itertools.product(
            degree_noise_list,
            offset_noise_list,
            time_noise_list,
            signal_noise_list,
            frequency_list,
            angular_velocity_deg_list,
            linear_velocity_cm_list,
        )
    )

    with progressbar.ProgressBar(max_value=len(combinations) * n_it) as p:
        for (
            degree_noise,
            offset_noise,
            time_noise,
            signal_noise,
            frequency,
            angular_velocity_deg,
            linear_velocity_cm,
        ) in combinations:

            if verbose:
                print(f"degree noise {degree_noise} of {degree_noise_list}")
                print(f"lateral noise {offset_noise} of {offset_noise_list}")
                print(f"time noise {time_noise} of {time_noise_list}")
                print(f"signal noise {signal_noise} of {signal_noise_list}")
                print(f"frequency {frequency} of {frequency_list}")
                print(
                    f"linear velocity {linear_velocity_cm} of {linear_velocity_cm_list}"
                )
                print(
                    f"angular velocity {angular_velocity_deg} of {angular_velocity_deg_list}"
                )

            for it in range(n_it):

                if verbose:
                    print(f"iteration {it + 1}/{n_it}")
                np.random.seed(seed)
                p.update(seed)
                seed += 1

                ##################### generate signals
                # create noisy versions
                degrees, offsets, times = get_movement(
                    angular_velocity_deg, linear_velocity_cm, n_samples, sampling_time
                )
                times_noisy = get_noisy_times(times, time_noise, time_quantization)

                # choose frequency bin (for now we make sure that the source signal is one of the available bins)
                f_idx = np.argmin(np.abs(frequencies_all - frequency))
                frequency_hz = frequencies_all[f_idx]
                frequencies = np.array(
                    [frequency_hz]
                )  # because algorithms expect multiple frequencies
                if abs(frequency_hz - frequency) > 100:
                    print("Warning: frequency mismatch", frequency_hz, frequency)

                mics_list_noisy = move_mics(
                    mics_local,
                    degrees,
                    offsets,
                    noise_dict={"degree": degree_noise, "offset": offset_noise},
                )

                ### generate signals at different positions
                signals_f_list = []
                for mics_noisy, time in zip(mics_list_noisy, times):
                    signals_received = generate_signals_analytical(
                        sources,
                        mics_noisy,
                        frequency_hz,
                        time,
                        noise=signal_noise,
                        phase_offset=phase_offsets,
                    )
                    buffer_ = signals_received[:, time_index : time_index + N_BUFFER]
                    signals_f = np.fft.rfft(buffer_).T
                    signals_f_list.append(signals_f[[f_idx], :])

                ### generate "real" multi-mic signals
                mics_array_noisy = np.concatenate([*mics_list_noisy])
                signals_multimic = generate_signals_analytical(
                    sources,
                    mics_array_noisy,
                    frequency_hz,
                    time=0,
                    noise=signal_noise,
                    phase_offset=phase_offsets,
                )
                buffer_multimic = signals_multimic[
                    :, time_index : time_index + N_BUFFER
                ]
                signals_f_multimic = (np.fft.rfft(buffer_multimic).T)[[f_idx], :]

                # print("mics_list_noisy", mics_list_noisy)
                # note that below we use the noiseless quantities for degrees, offsets, and times!
                # this is because we added the noise for generating the signals themselves.
                spectrum_dynamic, spectrum_delayed, spectrum_multimic = inner_loop(
                    mics_local,
                    degrees,
                    offsets,
                    times_noisy,
                    signals_f_list,
                    signals_f_multimic,
                    frequencies,
                )

                df.loc[len(df), :] = dict(
                    it=it,
                    seed=seed,
                    degree_noise=degree_noise,
                    offset_nosie=offset_noise,
                    time_noise=time_noise,
                    frequency=frequency_hz,
                    angular_velocity_deg=angular_velocity_deg,
                    linear_velocity_cm=linear_velocity_cm,
                    spectrum_dynamic=spectrum_dynamic,
                    spectrum_delayed=spectrum_delayed,
                    spectrum_multimic=spectrum_multimic,
                )

            if saveas != "":
                df.to_pickle(saveas)
                if verbose:
                    print(f"saved intermediate to {saveas}")
        return df


if __name__ == "__main__":
    import pandas as pd

    n_it = 20
    signal_noise_list = [1e-3]

    # movement noise study
    gt_angle_deg = 130  # angle of ground truth
    frequency_list = np.arange(1000, 5000, step=1000)
    angular_velocity_deg = [30]
    degree_noise_list = np.arange(0, 25, step=2)
    offset_noise_list = [0]
    time_noise_list = [0]
    n_samples = 2
    saveas = "results/doa_degree_noise.pkl"
    # simulate_doa(
    #   gt_angle_deg,
    #   degree_noise_list,
    #   offset_noise_list,
    #   signal_noise_list,
    #   time_noise_list,
    #   frequency_list,
    #   n_samples,
    #   n_it,
    #   angular_velocity_deg_list=angular_velocity_deg,
    #   saveas=saveas,
    # )

    # timestamp noise study
    frequency_list = np.arange(1000, 11000, step=1000)
    angular_velocity_deg = [30]
    degree_noise_list = [0]
    offset_noise_list = [0]
    time_noise_list = np.logspace(-6, 0, 20)
    n_samples = 2
    saveas = "results/doa_time_noise.pkl"
    # simulate_doa(
    #   gt_angle_deg,
    #   degree_noise_list,
    #   offset_noise_list,
    #   signal_noise_list,
    #   time_noise_list,
    #   frequency_list,
    #   n_samples,
    #   n_it,
    #   angular_velocity_deg_list=angular_velocity_deg,
    #   saveas=saveas,
    # )

    # lateral movement study
    gt_angle_deg = 50
    frequency_list = np.arange(1000, 11000, step=1000)
    angular_velocity_deg = [0.0]
    linear_velocity_cm = np.arange(300, step=10, dtype=float)
    degree_noise_list = [0.0]
    offset_noise_list = [1e-2]  # m/s
    time_noise_list = [0.0]
    n_samples = 2
    saveas = "results/doa_lateral_movement_highres.pkl"
    # simulate_doa(
    #    gt_angle_deg,
    #    degree_noise_list,
    #    offset_noise_list,
    #    signal_noise_list,
    #    time_noise_list,
    #    frequency_list,
    #    n_samples,
    #    n_it,
    #    linear_velocity_cm_list=linear_velocity_cm,
    #    saveas=saveas,
    # )

    # multiple sources
    n_it = 10
    gt_angle_deg = [10, 50]
    frequency_list = np.arange(1000, 11000, step=1000)
    angular_velocity_deg = [30.0]
    linear_velocity_cm = [10]
    degree_noise_list = np.arange(25, step=1)
    offset_noise_list = [0]  # m/s
    time_noise_list = [0.0]
    n_samples = 3
    # saveas = "results/doa_multi_joint_highres.pkl"
    saveas = ""  # "results/doa_multi_joint_highres.pkl"
    simulate_doa(
        gt_angle_deg,
        degree_noise_list,
        offset_noise_list,
        signal_noise_list,
        time_noise_list,
        frequency_list,
        n_samples,
        n_it,
        linear_velocity_cm_list=linear_velocity_cm,
        angular_velocity_deg_list=angular_velocity_deg,
        saveas=saveas,
    )
