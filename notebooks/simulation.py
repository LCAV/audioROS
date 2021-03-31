import sys

import numpy as np
import pyroomacoustics as pra

from audio_stack.beam_former import rotate_mics
from crazyflie_description_py.parameters import N_BUFFER, FS

from constants import SPEED_OF_SOUND
from frequency_analysis import get_bin
from geometry import *

sys.path.append("../crazyflie-audio/python")
from signals import generate_signal

# default wall absorption (percentage of amplitude that is lost in reflection):
WALL_ABSORPTION = 0.2
GAIN = 1.0
YAW_DEG = 0
ROOM_DIM = [10, 8] # in meters
WIDEBAND_FILE = "results/wideband.npy"

# default
N_TIMES = 10 # number of buffers to use for average (pyroomacoutics)

def create_wideband_signal(frequencies, duration_sec=1.0):
    phase = np.random.uniform(0, 2 * np.pi)
    kwargs = dict( 
        signal_type="mono",
        duration_sec=duration_sec,
        Fs=FS,
    )
    signal = generate_signal(
        frequency_hz=frequencies[1],
        phase_offset=phase,
        **kwargs
    )
    for f in frequencies[2:]:
        phase = np.random.uniform(0, 2 * np.pi)
        signal += generate_signal(
            frequency_hz=f, phase_offset=phase, **kwargs
        )
    return signal


def generate_room(distance_cm=0, yaw_deg=YAW_DEG, ax=None, fs_here=FS):

    # TODO(FD) -yaw_deg  because the wall moves relative to the yaw of the drone.
    # need to figure out a better notation.
    source, mic_positions = get_setup(distance_cm, -yaw_deg, ax)

    m = pra.Material(energy_absorption="glass_3mm")
    room = pra.ShoeBox(fs=fs_here, p=ROOM_DIM, max_order=1, materials=m)

    beam_former = pra.Beamformer(mic_positions.T, room.fs)
    room.add_microphone_array(beam_former)
    room.add_source(source)
    return room


def get_setup(distance_cm=0, yaw_deg=YAW_DEG, ax=None):
    context = Context.get_crazyflie_setup()

    d_wall_m = D_OFFSET + distance_cm * 1e-2
    offset = [ROOM_DIM[0] - d_wall_m, ROOM_DIM[1]/2]
    mic_positions = context.mics
    source = context.source + offset
    mic_positions = offset + rotate_mics(mic_positions, yaw_deg)

    if ax is not None:
        source_image = [source[0] + d_wall_m * 2, source[1]]
        for i, mic in enumerate(mic_positions):
            d = (ROOM_DIM[0] - mic[0]) * 100
            ax.scatter(*mic, label=f"mic{i}, d={d:.1f}cm", color=f"C{i}")
        ax.plot([ROOM_DIM[0], ROOM_DIM[0]], [0, ROOM_DIM[1]], label="wall", color="k")
        ax.scatter(*source, label="buzzer", color="C4")
        ax.scatter(*source_image, label="buzzer image", color="C4", marker="x")
        ymin = source[1] - 0.2
        ymax = source[1] + 0.2
        delta = (ymax - ymin) / 2
        ax.set_ylim(ymin, ymax)
        ax.set_xlim(source[0] - delta, source[0] + delta)
        #ax.axis('equal')
    return source, mic_positions


# ## simulation ###


def get_amplitude_function(
    distances_cm, gain, wall_absorption, mic_idx, yaw_deg=YAW_DEG
):
    deltas_m, d0 = get_deltas_from_global(yaw_deg, distances_cm, mic_idx)
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
    for _ in range(n_times-1):
        output_f = np.fft.rfft(
            room.mic_array.signals[:, idx : idx + n_buffer], axis=1
        ) # n_mics x n_frequencies
        h_f_list.append(output_f[None, :, :] / n_buffer)
        idx += n_buffer
    h_f = np.concatenate(h_f_list, axis=0)  # n_times x n_mics x n_frequencies
    return np.mean(np.abs(h_f), axis=0)


def get_df_theory(frequencies, distances, yaw_deg=YAW_DEG, chosen_mics=range(4)):
    H = np.zeros((len(chosen_mics), len(frequencies), len(distances)))
    for i, mic in enumerate(chosen_mics):
        deltas_m, d0 = get_deltas_from_global(yaw_deg=yaw_deg, distances_cm=distances, mic_idx=mic)
        H[i, :, :] = get_df_theory_simple(deltas, frequencies, d0).T
    return H


def get_freq_slice_pyroom(frequencies, distance_cm, signal, yaw_deg=YAW_DEG):
    import pandas as pd

    room = generate_room(distance_cm=distance_cm, yaw_deg=yaw_deg)

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


def get_dist_slice_pyroom(frequency, distances_cm, yaw_deg=YAW_DEG, n_times=100):
    from frequency_analysis import get_bin

    duration_sec = N_BUFFER * n_times / FS
    signal = generate_signal(
        FS, duration_sec=duration_sec, signal_type="mono", frequency_hz=frequency,
    )
    freqs_all = np.fft.rfftfreq(N_BUFFER, 1 / FS)
    bin_ = get_bin(freqs_all, frequency)

    Hs = []
    for d in distances_cm:
        room = generate_room(distance_cm=d, yaw_deg=yaw_deg)
        mag = get_average_magnitude(room, signal, n_buffer=N_BUFFER, n_times=n_times)
        Hs.append(mag[:, bin_] ** 2)
    return np.array(Hs)


def get_freq_slice_theory(
    frequencies, distance_cm, yaw_deg=YAW_DEG, chosen_mics=range(4)
):
    """ 
    We can incorporate relative movement by providing
    distance_cm and yaw_deg of same length as frequencies. 
    """
    Hs = np.zeros((len(frequencies), len(chosen_mics)))
    for i, mic in enumerate(chosen_mics):
        deltas_m, d0 = get_deltas_from_global(yaw_deg=yaw_deg, distances_cm=distance_cm, mic_idx=mic)
        pattern = get_df_theory_simple(deltas_m, frequencies, d0, flat=True)
        Hs[:, i] = pattern
    return Hs


def get_dist_slice_theory(
    frequency,
    distances_cm,
    yaw_deg=YAW_DEG,
    chosen_mics=range(4),
    wall_absorption=WALL_ABSORPTION,
    gains=[GAIN] * 4,
):
    """ 
    We can incorporate relative movement by providing
    distance_cm and yaw_deg of same length as frequencies. 
    """
    if np.ndim(gains) == 0:
        gains = [gains] * len(chosen_mics)
    elif len(gains) == 1:
        gains = [gains[0]] * len(chosen_mics)

    Hs = np.zeros((len(distances_cm), len(chosen_mics)))
    for i, mic in enumerate(chosen_mics):
        deltas_m, d0 = get_deltas_from_global(yaw_deg, distances_cm, mic)
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


def factor_distance_to_delta(distance, mic):
    delta_m, d0 = get_deltas_from_global(0, distance, mic)
    return delta_m * 1e2 / distance
