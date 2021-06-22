import sys

import numpy as np
import pyroomacoustics as pra
from audio_stack.beam_former import rotate_mics
from constants import SPEED_OF_SOUND
from crazyflie_description_py.parameters import N_BUFFER, FS
from crazyflie_description_py.experiments import WALL_ANGLE_DEG, ROOM_DIM
from frequency_analysis import get_bin
from geometry import *

sys.path.append("../crazyflie-audio/python")
from signals import generate_signal

# default wall absorption (percentage of amplitude that is lost in reflection):
WALL_ABSORPTION = 0.2
GAIN = 1.0  # default amplitude for input signals
WIDEBAND_FILE = "results/wideband.npy"
N_TIMES = 10  # number of buffers to use for average (pyroomacoutics)


def simulate_distance_estimator(
    chosen_mics=range(4), distance_cm=10, azimuth_deg=WALL_ANGLE_DEG, ax=None
):
    from inference import get_probability_bayes
    from estimators import DistanceEstimator

    n_max = 1000
    frequencies = np.linspace(1000, 5000, 32)
    slices_f = get_freq_slice_theory(
        frequencies, distance_cm, chosen_mics=chosen_mics, azimuth_deg=azimuth_deg
    )

    distance_estimator = DistanceEstimator()
    for i, mic_idx in enumerate(chosen_mics):
        slice_f = slices_f[:, i]
        d_bayes, p_bayes, diff_cm = get_probability_bayes(
            slice_f, frequencies, n_max=n_max, azimuth_deg=azimuth_deg
        )
        distance_estimator.add_distribution(diff_cm * 1e-2, p_bayes, mic_idx)
        if ax is not None:
            ax.scatter(diff_cm, p_bayes, color=f"C{mic_idx}", label=f"mic{mic_idx}")

    if ax is not None:
        ax.set_xlabel("path difference [cm]")
        ax.set_ylabel("probability [-]")
        ax.legend()
    return distance_estimator


def create_wideband_signal(frequencies, duration_sec=1.0):
    phase = np.random.uniform(0, 2 * np.pi)
    kwargs = dict(signal_type="mono", duration_sec=duration_sec, Fs=FS,)
    signal = generate_signal(frequency_hz=frequencies[1], phase_offset=phase, **kwargs)
    for f in frequencies[2:]:
        phase = np.random.uniform(0, 2 * np.pi)
        signal += generate_signal(frequency_hz=f, phase_offset=phase, **kwargs)
    return signal


def generate_room(distance_cm=0, azimuth_deg=WALL_ANGLE_DEG, ax=None, fs_here=FS):
    """ Generate two-dimensional setup using pyroomacoustics. """
    source, mic_positions = get_setup(distance_cm, azimuth_deg, ax)

    m = pra.Material(energy_absorption="glass_3mm")
    room = pra.ShoeBox(fs=fs_here, p=ROOM_DIM[:2], max_order=1, materials=m)

    beam_former = pra.Beamformer(mic_positions.T, room.fs)
    room.add_microphone_array(beam_former)
    room.add_source(source)
    return room


def get_setup(distance_cm=0, azimuth_deg=WALL_ANGLE_DEG, ax=None, zoom=True):
    """ Create a setup for pyroomacoustics that corresponds to distance_cm and azimuth_deg"""
    context = Context.get_crazyflie_setup()

    d_wall_m = distance_cm * 1e-2  # distance of wall
    offset = [ROOM_DIM[0] - d_wall_m, ROOM_DIM[1] / 2]  # location of drone
    mic_positions = context.mics
    source = context.source + offset

    # note that we need to take the negative azimuth, because the drone has to
    # be moved in the opposite direction.
    mic_positions = offset + rotate_mics(mic_positions, -azimuth_deg)

    if ax is not None:
        source_image = [source[0] + d_wall_m * 2, source[1]]
        for i, mic in enumerate(mic_positions):
            d = (ROOM_DIM[0] - mic[0]) * 100
            ax.scatter(*mic, label=f"mic{i}, d={d:.1f}cm", color=f"C{i}")
        ax.axvline(x=ROOM_DIM[0], label="wall", color="k")
        ax.scatter(*source, label="buzzer", color="C4")
        ax.scatter(*source_image, label="buzzer image", color="C4", marker="x")
        if not zoom:
            ax.axhline(y=ROOM_DIM[1], color="k")
            ax.axvline(x=0, color="k")
            ax.axhline(y=0, color="k")
        else:
            xmin = min([min(mic_positions[:, 0]), source[0], source_image[0]])
            xmax = max([max(mic_positions[:, 0]), source[0], source_image[0]])
            delta = (xmax - xmin) / 4
            ax.set_xlim(xmin - delta, xmax + delta)
            ax.axis("equal")
    return source, mic_positions


# ## simulation ###


def get_amplitude_function(
    distances_cm, gain, wall_absorption, mic_idx, azimuth_deg=WALL_ANGLE_DEG
):
    deltas_m, d0 = get_deltas_from_global(azimuth_deg, distances_cm, mic_idx)
    alpha0 = 1 / (4 * np.pi * d0)  #
    alpha1 = (1 - wall_absorption) / (4 * np.pi * (deltas_m + d0))
    amplitudes = gain * 2 * alpha0 * alpha1
    return amplitudes


def get_df_theory_simple(
    deltas_m,
    frequencies_hz,
    d0,
    flat=False,
    wall_absorption=WALL_ABSORPTION,
    gain=GAIN,
    c=SPEED_OF_SOUND,
):

    alpha0 = 1 / (4 * np.pi * d0)  #
    alpha1 = (1 - wall_absorption) / (4 * np.pi * (deltas_m + d0))

    frequencies_hz = np.array(frequencies_hz)
    deltas_m = np.array(deltas_m)
    if not flat:
        deltas_m = np.array(deltas_m).reshape((-1, 1))
        alpha1 = alpha1.reshape((-1, 1))
        frequencies_hz = np.array(frequencies_hz).reshape((1, -1))

    mag_squared = (
        alpha0 ** 2
        + alpha1 ** 2
        + 2 * alpha0 * alpha1 * np.cos(2 * np.pi * frequencies_hz * deltas_m / c)
    )  # n_deltas x n_freqs or n_freqs
    return mag_squared * gain


def get_average_magnitude(room, signal, n_buffer=N_BUFFER, n_times=N_TIMES):
    """ 
    :param signal: signal to use for simulation, array
    :param n_buffer: buffer size for "STFT"
    :param n_times: number of buffers to average. Will start fromsecond buffer.
    """
    assert len(room.sources) == 1

    room.sources[0].add_signal(signal)
    room.simulate()

    assert (n_times * n_buffer) < room.mic_array.signals.shape[
        1
    ], f"{n_times}*{n_buffer}={n_times * n_buffer}, {room.mic_array.signals.shape}"

    h_f_list = []
    idx = n_buffer  # skip first buffer to avoid boundary effects
    for _ in range(n_times - 1):
        output_f = np.fft.rfft(
            room.mic_array.signals[:, idx : idx + n_buffer], axis=1
        )  # n_mics x n_frequencies
        h_f_list.append(output_f[None, :, :] / n_buffer)
        idx += n_buffer
    h_f = np.concatenate(h_f_list, axis=0)  # n_times x n_mics x n_frequencies
    return np.mean(np.abs(h_f), axis=0)


def get_df_theory(
    frequencies, distances, azimuth_deg=WALL_ANGLE_DEG, chosen_mics=range(4)
):
    H = np.zeros((len(chosen_mics), len(frequencies), len(distances)))
    for i, mic in enumerate(chosen_mics):
        deltas_m, d0 = get_deltas_from_global(
            azimuth_deg=azimuth_deg, distances_cm=distances, mic_idx=mic
        )
        H[i, :, :] = get_df_theory_simple(deltas_m, frequencies, d0).T
    return H


def get_freq_slice_pyroom(frequencies, distance_cm, signal, azimuth_deg=WALL_ANGLE_DEG):
    room = generate_room(distance_cm=distance_cm, azimuth_deg=azimuth_deg)

    n_times = len(signal) // N_BUFFER
    mag = get_average_magnitude(
        room, signal, n_buffer=N_BUFFER, n_times=n_times
    )  # n_mics x n_frequencies
    freqs_all = np.fft.rfftfreq(N_BUFFER, 1 / FS)
    if len(frequencies) < len(freqs_all):
        bins_ = [get_bin(freqs_all, f) for f in frequencies]
    else:
        bins_ = np.arange(len(frequencies))
    return mag[:, bins_] ** 2


def get_dist_slice_pyroom(
    frequency, distances_cm, azimuth_deg=WALL_ANGLE_DEG, n_times=100
):
    from frequency_analysis import get_bin

    duration_sec = N_BUFFER * n_times / FS
    signal = generate_signal(
        FS, duration_sec=duration_sec, signal_type="mono", frequency_hz=frequency,
    )
    freqs_all = np.fft.rfftfreq(N_BUFFER, 1 / FS)
    bin_ = get_bin(freqs_all, frequency)

    Hs = []
    for d in distances_cm:
        room = generate_room(distance_cm=d, azimuth_deg=azimuth_deg)
        mag = get_average_magnitude(room, signal, n_buffer=N_BUFFER, n_times=n_times)
        Hs.append(mag[:, bin_] ** 2)
    return np.array(Hs)


def get_freq_slice_theory(
    frequencies, distance_cm, azimuth_deg=WALL_ANGLE_DEG, chosen_mics=range(4)
):
    """ 
    We can incorporate relative movement by providing
    distance_cm and azimuth_deg of same length as frequencies. 
    """
    Hs = np.zeros((len(frequencies), len(chosen_mics)))
    for i, mic in enumerate(chosen_mics):
        deltas_m, d0 = get_deltas_from_global(
            azimuth_deg=azimuth_deg, distances_cm=distance_cm, mic_idx=mic
        )
        pattern = get_df_theory_simple(deltas_m, frequencies, d0, flat=True)
        Hs[:, i] = pattern
    return Hs


def get_dist_slice_theory(
    frequency,
    distances_cm,
    azimuth_deg=WALL_ANGLE_DEG,
    chosen_mics=range(4),
    wall_absorption=WALL_ABSORPTION,
    gains=[GAIN] * 4,
):
    """ 
    We can incorporate relative movement by providing
    distance_cm and azimuth_deg of same length as frequencies. 
    """
    if np.ndim(gains) == 0:
        gains = [gains] * len(chosen_mics)
    elif len(gains) == 1:
        gains = [gains[0]] * len(chosen_mics)

    Hs = np.zeros((len(distances_cm), len(chosen_mics)))
    for i, mic in enumerate(chosen_mics):
        deltas_m, d0 = get_deltas_from_global(azimuth_deg, distances_cm, mic)
        pattern = get_df_theory_simple(
            deltas_m,
            [frequency],
            d0,
            flat=True,
            wall_absorption=wall_absorption,
            gain=gains[i],
            c=SPEED_OF_SOUND,
        )
        Hs[:, i] = pattern.flatten()
    return Hs


def factor_distance_to_delta(d1_cm, rel_movement_cm, mic, azimuth_deg=0):
    delta_d1, d0 = get_deltas_from_global(azimuth_deg, d1_cm, mic)
    delta_d2, d0 = get_deltas_from_global(azimuth_deg, d1_cm - rel_movement_cm, mic)
    return (delta_d1 - delta_d2) * 1e2 / rel_movement_cm
