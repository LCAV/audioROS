import sys

import matplotlib.pylab as plt
import numpy as np
import pyroomacoustics as pra

from audio_stack.beam_former import rotate_mics
from constants import SPEED_OF_SOUND
from crazyflie_description_py.parameters import MIC_POSITIONS, FS, N_BUFFER

sys.path.append('../crazyflie-audio/python')
from signals import generate_signal, amplify_signal

DURATION_SEC = 38
N_TIMES = DURATION_SEC * FS // (N_BUFFER * 2)
D0 = 0.1 # distance between mic and source for single-mic case, in meters

# this corresponds to the setup in BC325: 
Y_OFFSET = 0.08
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
            ax.scatter(*mic, label=f'mic{i}, d={d:.1f}cm', color=f'C{i}')
        ax.plot([0, ROOM_DIM[0]], [0, 0], label='wall', color='k')
        ax.scatter(*source, label='buzzer', color='C4')
        ax.scatter(*source_image, label='buzzer image', color='C4', marker='x')
        ymin = -source[1] - 0.5
        ymax = source[1] + 0.5
        delta = (ymax-ymin)/2
        ax.set_ylim(ymin, ymax)
        ax.set_xlim(source[0] - delta, source[0] + delta)
        #ax.axis('equal')

    return source, mic_positions


def generate_room(distance_cm=0, yaw_deg=0, ax=None, single_mic=False, fs_here=FS):
    source, mic_positions = get_setup(distance_cm, yaw_deg, ax, single_mic)

    m = pra.Material(energy_absorption="glass_3mm")
    room = pra.ShoeBox(fs=fs_here, p=ROOM_DIM, max_order=1, materials=m)
        
    beam_former = pra.Beamformer(mic_positions.T,  room.fs)
    room.add_microphone_array(beam_former)
    room.add_source(source)
    return room


def get_signals_f(room, signal, n_buffer=N_BUFFER, n_times=N_TIMES):
    assert len(room.sources) == 1
    room.sources[0].add_signal(signal)
    room.simulate()
    
    assert (n_times * n_buffer) < room.mic_array.signals.shape[1], f"{n_times}*{n_buffer}={n_times * n_buffer}, {room.mic_array.signals.shape}"
    
    signals_f_list = []
    idx = 0
    for _ in range(n_times):
        signals_f = np.fft.rfft(room.mic_array.signals[:, idx:idx+n_buffer], axis=1)
        signals_f_list.append(signals_f[None, ...])
        idx += n_buffer
    signals_f = np.concatenate(signals_f_list, axis=0)
    return signals_f


def generate_rir(frequencies, distance_cm=0, yaw_deg=0, single_mic=False, ax=None, gain=1.0):
    """ 
    Generate one-wall RIR using analytical formula. 

    Returns list of length n_mics, where each element contains n_frequencies complex values.
    """
    source, mic_positions = get_setup(distance_cm, yaw_deg, ax, single_mic)
    source_image = [source[0], -source[1]]

    # simplification1: attenuation normally depends on pressure, humidity, frequency.
    # simplification 2: no extra loss at wall
    a = 10 # attenuation coefficient in dB / m, between 100-200 degrees, 4000+ Hz
    Hs = []
    for mic in mic_positions:
        direct_path = np.linalg.norm(mic - source)
        reflect_path = np.linalg.norm(mic - source_image)
        alpha0 = 10**(-a*direct_path/20)
        alpha1 = 10**(-a*reflect_path/20)
        n0 = direct_path / SPEED_OF_SOUND
        n1 = reflect_path / SPEED_OF_SOUND
        H_ij = alpha0 * gain * np.exp(-1j*2*np.pi*frequencies*n0) + alpha1 * gain * np.exp(-1j*2*np.pi*frequencies*n1)
        Hs.append(H_ij)
    return Hs


def get_freq_slice_pyroom(frequencies, distance_cm, yaw_deg=0, signal=None):
    import pandas as pd
    room = generate_room(distance_cm=distance_cm, single_mic=False)

    # TODO(fd) ideally we would always generate a signal at the given frequencies only,
    # as shown below.
    # however, this takes long for many frequencies, so we don't do that and use
    # a precomputed version, which as all frequencies present (but potentially not
    # exactly the ones given to the function...) 

    #duration_sec = 10
    #n_samples = fs * duration_sec 
    #signal = np.zeros(n_samples)
    #for f in frequencies:
    #    phase = np.random.uniform(0, 2*np.pi)
    #    signal += generate_signal(signal_type='mono', frequency_hz=f, duration_sec=10, fs=fs, 
    #            max_db=-10,
    #            phase_offset=phase)
    if signal is None:
        signal = pd.read_pickle('results/multi.pk')

    n_times = len(signal) // N_BUFFER
    signals_f = get_signals_f(room, signal, n_buffer=N_BUFFER, n_times=n_times) # n_buffer=n_buffer, n_times=5)
    freqs_all = np.fft.rfftfreq(N_BUFFER, 1/FS)
    if len(frequencies) < len(freqs_all):
        bins_ = [np.argmin(np.abs(f - freqs_all)) for f in frequencies]
    else:
        bins_ = np.arange(len(frequencies))
    return np.mean(np.abs(signals_f[:, :, bins_]), axis=0) / N_BUFFER


def get_freq_slice_theory(frequencies, distance_cm, yaw_deg=0):
    Hs = generate_rir(frequencies, distance_cm, yaw_deg, single_mic=False, ax=None)
    return [np.abs(H) for H in Hs]


def get_dist_slice_pyroom(frequency, distances_cm, yaw_deg=0, n_times=100):
    import pandas as pd
    duration_sec = N_BUFFER * n_times / FS
    signal = generate_signal(FS, duration_sec=duration_sec, signal_type="mono", frequency_hz=frequency)
    freqs_all = np.fft.rfftfreq(N_BUFFER, 1/FS)
    bin_ = np.argmin(np.abs(freqs_all - frequency))

    Hs = []
    for d in distances_cm:
        room = generate_room(distance_cm=d, single_mic=False)
        signals_f = get_signals_f(room, signal, n_buffer=N_BUFFER, n_times=n_times) # n_buffer=n_buffer, n_times=5)
        Hs.append(np.mean(np.abs(signals_f[:, :, bin_]), axis=0) / N_BUFFER)
    return np.array(Hs)


def get_dist_slice_theory(frequency, distances_cm, yaw_deg=0):
    Hs = []
    for d in distances_cm:
        H = generate_rir(np.array([frequency]), d, yaw_deg, single_mic=False, ax=None, gain=5)
        H = np.array(H)
        assert H.shape[1] == 1, H.shape
        Hs.append(np.abs(H[:, 0]))
    return np.array(Hs)
