import sys

import numpy as np
import pyroomacoustics as pra

from audio_stack.beam_former import rotate_mics
from constants import SPEED_OF_SOUND
from crazyflie_description_py.parameters import (
    MIC_POSITIONS,
    BUZZER_POSITION,
    FS,
    N_BUFFER,
)
from frequency_analysis import get_bin

sys.path.append("../crazyflie-audio/python")
from signals import generate_signal

DURATION_SEC = 38
N_TIMES = DURATION_SEC * FS // (N_BUFFER * 2)
# default difference between mic and speaker, in meters:
D0 = 0.1
# default wall absorption (percentage of amplitude that is lost in reflection):
WALL_ABSORPTION = 0.2
GAIN = 1.0

# this corresponds to the setup in BC325 with stepper motor:
Y_OFFSET = 0.08  # in meters
YAW_OFFSET = -132  # in degrees
ROOM_DIM = [10, 8]

### geometry ###


def get_delta(distance_cm, beta_rad, d0_cm=5):
    """
    :param distance_cm: orthogonal distance from wall
    :param beta_rad: angle between mic-source and wall normal
    :return: difference in path length between reflection and direct path and 
    """
    distance_cm = (
        np.array([distance_cm]) if type(distance_cm) != np.ndarray else distance_cm
    )
    beta_rad = np.array([beta_rad]) if type(beta_rad) != np.ndarray else beta_rad
    d1_cm = np.sqrt(
        d0_cm ** 2 + 4 * distance_cm ** 2 - 4 * distance_cm * d0_cm * np.cos(beta_rad)
    )
    return d1_cm - d0_cm


def get_orthogonal_distance(delta_cm, beta_rad, d0_cm=5):
    """
    :param delta_cm: difference in path length between reflection and direct path
    :param beta_rad: angle between mic-source and wall normal
    :return: orthogonal distance from wall
    """
    delta_cm = np.array([delta_cm]) if type(delta_cm) != np.ndarray else delta_cm
    beta_rad = np.array([beta_rad]) if type(beta_rad) != np.ndarray else beta_rad
    d1 = delta_cm + d0_cm
    d0cos = d0_cm * np.cos(beta_rad)
    d_cm = 0.5 * (
        d0cos[:, None] + np.sqrt(d0cos[:, None] ** 2 + d1[None, :] ** 2 - d0_cm ** 2)
    )
    return d_cm


def get_setup(distance_cm=0, yaw_deg=0, ax=None):
    offset = [ROOM_DIM[0] / 2, Y_OFFSET + distance_cm * 1e-2]
    mic_positions = np.array(MIC_POSITIONS)
    source = np.array(BUZZER_POSITION).flatten() + offset
    mic_positions = offset + rotate_mics(mic_positions, YAW_OFFSET - yaw_deg)

    if ax is not None:
        source_image = [source[0], -source[1]]
        for i, mic in enumerate(mic_positions):
            d = mic[1] * 100
            ax.scatter(*mic, label=f"mic{i}, d={d:.1f}cm", color=f"C{i}")
        ax.plot([0, ROOM_DIM[0]], [0, 0], label="wall", color="k")
        ax.scatter(*source, label="buzzer", color="C4")
        ax.scatter(*source_image, label="buzzer image", color="C4", marker="x")
        ymin = -source[1] - 0.5
        ymax = source[1] + 0.5
        delta = (ymax - ymin) / 2
        ax.set_ylim(ymin, ymax)
        ax.set_xlim(source[0] - delta, source[0] + delta)
        # ax.axis('equal')

    return source, mic_positions


def get_deltas_from_global(yaw_deg, distances_cm, mic_idx, ax=None):
    mic = np.array(MIC_POSITIONS)[mic_idx, :2]
    source = np.array(BUZZER_POSITION)[:2]
    vec = (mic - source).flatten()
    d0_cm = np.linalg.norm(vec) * 100

    distances_here = np.array(distances_cm) + (Y_OFFSET * 100)
    yaw_here = (YAW_OFFSET - np.array(yaw_deg)) / 180 * np.pi
    beta_here = np.arctan2(vec[1], vec[0]) + np.pi / 2 + yaw_here

    deltas = get_delta(distances_here, beta_here, d0_cm=d0_cm).flatten() * 1e-2
    if ax is not None:
        ax.plot(
            [0, d0_cm * np.cos(beta_here)],
            [0, d0_cm * np.sin(beta_here)],
            color=f"C{mic_idx}",
            label=f"mic{mic_idx}",
            marker="o",
        )
    return deltas, d0_cm * 1e-2


def get_orthogonal_distance_from_global(yaw_deg, deltas_cm, mic_idx, ax=None):
    mic = np.array(MIC_POSITIONS)[mic_idx, :2]
    source = np.array(BUZZER_POSITION)[:2]
    vec = (mic - source).flatten()
    d0_cm = np.linalg.norm(vec) * 100

    yaw_here = (YAW_OFFSET - np.array(yaw_deg)) / 180 * np.pi
    beta_here = np.arctan2(vec[1], vec[0]) + np.pi / 2 + yaw_here

    distances_cm = get_orthogonal_distance(deltas_cm, beta_here, d0_cm=d0_cm).flatten()
    distances_here = np.array(distances_cm) - (Y_OFFSET * 100)
    return distances_here


def generate_room(distance_cm=0, yaw_deg=0, ax=None, fs_here=FS):
    source, mic_positions = get_setup(distance_cm, yaw_deg, ax)

    m = pra.Material(energy_absorption="glass_3mm")
    room = pra.ShoeBox(fs=fs_here, p=ROOM_DIM, max_order=1, materials=m)

    beam_former = pra.Beamformer(mic_positions.T, room.fs)
    room.add_microphone_array(beam_former)
    room.add_source(source)
    return room


### simulation ###


def get_df_theory_simple(
    deltas_m,
    frequencies_hz,
    flat=False,
    d0=D0,
    wall_absorption=WALL_ABSORPTION,
    gain_x=GAIN,
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
    return mag_squared * gain_x ** 2


def get_average_magnitude(room, signal, n_buffer=N_BUFFER, n_times=N_TIMES):
    assert len(room.sources) == 1

    room.sources[0].add_signal(signal)
    room.simulate()

    assert (n_times * n_buffer) < room.mic_array.signals.shape[
        1
    ], f"{n_times}*{n_buffer}={n_times * n_buffer}, {room.mic_array.signals.shape}"

    h_f_list = []
    idx = 0
    for _ in range(n_times):
        input_f = np.fft.rfft(signal[idx : idx + n_buffer])
        output_f = np.fft.rfft(
            room.mic_array.signals[:, idx : idx + n_buffer], axis=1
        )  # n_mics x n_frequencies
        ratio_f = output_f[None, ...] / input_f[None, None, :]

        # TODO(FD) this ratio is not between 0 and 1. Find out if this is normal.
        # print('ratio:', np.min(np.abs(ratio_f)), np.max(np.abs(ratio_f)))
        h_f_list.append(ratio_f)
        idx += n_buffer
    h_f = np.concatenate(h_f_list, axis=0)  # n_times x n_mics x n_frequencies

    start_idx = n_times // 5  # ignore first few buffers because of border effects.
    return np.mean(np.abs(h_f[start_idx:]), axis=0)


def get_df_theory(frequencies, distances, chosen_mics=range(4)):
    H = np.zeros((len(chosen_mics), len(frequencies), len(distances)))
    for i, mic in enumerate(chosen_mics):
        deltas, d0 = get_deltas_from_global(
            yaw_deg=0, distances_cm=distances, mic_idx=mic
        )
        H[i, :, :] = get_df_theory_simple(deltas, frequencies, d0=d0).T
    return H


def get_freq_slice_pyroom(frequencies, distance_cm, yaw_deg=0, signal=None):
    import pandas as pd

    room = generate_room(distance_cm=distance_cm)

    # TODO(fd) ideally we would always generate a signal at the given frequencies only,
    # as shown below.
    # however, this takes long for many frequencies, so we don't do that and use
    # a precomputed version, which as all frequencies present (but potentially not
    # exactly the ones given to the function...)
    if signal is None:
        try:
            signal = pd.read_pickle("results/multi.pk")
        except FileNotFoundError:
            print("Run WallStudy notebook to save results/multi.pk")

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


def get_dist_slice_pyroom(frequency, distances_cm, yaw_deg=0, n_times=100):
    from frequency_analysis import get_bin

    if frequency > 0:
        duration_sec = N_BUFFER * n_times / FS
        signal = generate_signal(
            FS, duration_sec=duration_sec, signal_type="mono", frequency_hz=frequency
        )
    else:
        signal = np.zeros(n_times * N_BUFFER)
    freqs_all = np.fft.rfftfreq(N_BUFFER, 1 / FS)

    bin_ = get_bin(freqs_all, frequency)

    Hs = []
    for d in distances_cm:
        room = generate_room(distance_cm=d, yaw_deg=yaw_deg)
        mag = get_average_magnitude(room, signal, n_buffer=N_BUFFER, n_times=n_times)
        Hs.append(mag[:, bin_] ** 2)
    return np.array(Hs)


def get_freq_slice_theory(frequencies, distance_cm, yaw_deg=0, chosen_mics=range(4)):
    """ 
    We can incorporate relative movement by providing
    distance_cm and yaw_deg of same length as frequencies. 
    """
    Hs = np.zeros((len(frequencies), len(chosen_mics)))
    for i, mic in enumerate(chosen_mics):
        deltas_m, d0 = get_deltas_from_global(yaw_deg, distance_cm, mic)
        pattern = get_df_theory_simple(deltas_m, frequencies, flat=True, d0=d0)
        Hs[:, i] = pattern
    return Hs


def get_dist_slice_theory(
    frequency,
    distances_cm,
    yaw_deg=0,
    chosen_mics=range(4),
    wall_absorption=WALL_ABSORPTION,
    gain_x=GAIN,
):
    """ 
    We can incorporate relative movement by providing
    distance_cm and yaw_deg of same length as frequencies. 
    """
    Hs = np.zeros((len(distances_cm), len(chosen_mics)))
    for i, mic in enumerate(chosen_mics):
        deltas_m, d0 = get_deltas_from_global(yaw_deg, distances_cm, mic)
        pattern = get_df_theory_simple(
            deltas_m,
            [frequency],
            flat=True,
            d0=d0,
            wall_absorption=wall_absorption,
            gain_x=gain_x,
            c=SPEED_OF_SOUND,
        )
        Hs[:, i] = pattern.flatten()
    return Hs
