import sys

import numpy as np
import pyroomacoustics as pra

from audio_stack.beam_former import rotate_mics
from constants import SPEED_OF_SOUND
from crazyflie_description_py.parameters import MIC_POSITIONS, FS, N_BUFFER
from frequency_analysis import get_bin

sys.path.append("../crazyflie-audio/python")
from signals import generate_signal

DURATION_SEC = 38
N_TIMES = DURATION_SEC * FS // (N_BUFFER * 2)
D0 = 0.1  # distance between mic and source for single-mic case, in meters
ATTENUATION = (
    20  # attenuation coefficient in dB / m, 10: between 100-200 degrees, 4000+ Hz
)

# this corresponds to the setup in BC325:
Y_OFFSET = 0.08  # in meters
YAW_OFFSET = -132
ROOM_DIM = [10, 8]


def get_setup(distance_cm=0, yaw_deg=0, ax=None, single_mic=False):
    source = [ROOM_DIM[0] / 2, Y_OFFSET + distance_cm * 1e-2]
    if single_mic:
        mic_positions = np.array([source[0] + D0, source[1]]).reshape((1, len(source)))
    else:
        mic_positions = np.array(MIC_POSITIONS)
        mic_positions = source + rotate_mics(mic_positions, YAW_OFFSET - yaw_deg)

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


def get_real_distance(distance_cm=0, yaw_deg=0, mic_idx=0):
    source, mic_positions = get_setup(
        distance_cm=distance_cm, yaw_deg=yaw_deg, single_mic=False
    )
    source_image = [source[0], -source[1]]
    mic = mic_positions[mic_idx]
    return 100 * (np.linalg.norm(source_image - mic) - np.linalg.norm(source - mic))


def generate_room(distance_cm=0, yaw_deg=0, ax=None, single_mic=False, fs_here=FS):
    source, mic_positions = get_setup(distance_cm, yaw_deg, ax, single_mic)

    m = pra.Material(energy_absorption="glass_3mm")
    room = pra.ShoeBox(fs=fs_here, p=ROOM_DIM, max_order=1, materials=m)

    beam_former = pra.Beamformer(mic_positions.T, room.fs)
    room.add_microphone_array(beam_former)
    room.add_source(source)
    return room


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


def get_slice_theory(mic_positions, source, frequencies, gain=1.0):
    # simplification1: attenuation normally depends on pressure, humidity, frequency.
    # simplification 2: no extra loss at wall
    source_image = [source[0], -source[1]]
    n_mics = mic_positions.shape[0]
    H = np.empty((n_mics, len(frequencies)))
    for i, mic in enumerate(mic_positions):
        direct_path = np.linalg.norm(mic - source)
        reflect_path = np.linalg.norm(mic - source_image)
        alpha0 = 10 ** (-ATTENUATION * direct_path / 20)
        alpha1 = 10 ** (-ATTENUATION * reflect_path / 20)
        n0 = direct_path / SPEED_OF_SOUND
        n1 = reflect_path / SPEED_OF_SOUND
        H_ij = alpha0 * gain * np.exp(
            -1j * 2 * np.pi * frequencies * n0
        ) + alpha1 * gain * np.exp(-1j * 2 * np.pi * frequencies * n1)
        H[i, :] = np.abs(H_ij)
    return H


def get_df_theory(frequencies, distances, gain=1.0, chosen_mics=range(4)):
    """ 
    :param gain: can also be a function of frequency, of same length as frequencies.
    """
    n_mics = len(chosen_mics)
    H = np.zeros((n_mics, len(frequencies), len(distances)))
    for k, distance in enumerate(distances):
        source, mic_positions = get_setup(
            distance, yaw_deg=0, ax=None, single_mic=False
        )
        H[:, :, k] = get_slice_theory(
            mic_positions[chosen_mics], source, frequencies, gain=gain
        )
    return H


def get_freq_slice_pyroom(frequencies, distance_cm, yaw_deg=0, signal=None):
    import pandas as pd

    room = generate_room(distance_cm=distance_cm, single_mic=False)

    # TODO(fd) ideally we would always generate a signal at the given frequencies only,
    # as shown below.
    # however, this takes long for many frequencies, so we don't do that and use
    # a precomputed version, which as all frequencies present (but potentially not
    # exactly the ones given to the function...)

    # duration_sec = 10
    # n_samples = fs * duration_sec
    # signal = np.zeros(n_samples)
    # for f in frequencies:
    #    phase = np.random.uniform(0, 2*np.pi)
    #    signal += generate_signal(signal_type='mono', frequency_hz=f, duration_sec=10, fs=fs,
    #            max_db=-10,
    #            phase_offset=phase)
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
    return mag[:, bins_]


def get_freq_slice_theory(frequencies, distance_cm, yaw_deg=0):
    source, mic_positions = get_setup(distance_cm, yaw_deg, ax=None, single_mic=False)
    Hs = get_slice_theory(mic_positions, source, frequencies, gain=1.0)
    return [np.abs(H) for H in Hs]


def get_dist_slice_pyroom(frequency, distances_cm, yaw_deg=0, n_times=100):
    from frequency_analysis import get_bin

    duration_sec = N_BUFFER * n_times / FS
    signal = generate_signal(
        FS, duration_sec=duration_sec, signal_type="mono", frequency_hz=frequency
    )
    freqs_all = np.fft.rfftfreq(N_BUFFER, 1 / FS)

    bin_ = get_bin(freqs_all, frequency)

    Hs = []
    for d in distances_cm:
        room = generate_room(distance_cm=d, single_mic=False)
        mag = get_average_magnitude(room, signal, n_buffer=N_BUFFER, n_times=n_times)
        Hs.append(mag[:, bin_])
    return np.array(Hs)


def get_dist_slice_theory(frequency, distances_cm, yaw_deg=0, gain=1.0):
    Hs = []
    for d in distances_cm:
        source, mic_positions = get_setup(d, yaw_deg, ax=None, single_mic=False)
        H = get_slice_theory(mic_positions, source, np.array([frequency]), gain=gain)
        H = np.array(H)
        assert H.shape[1] == 1, H.shape
        Hs.append(np.abs(H[:, 0]))
    return np.array(Hs)
