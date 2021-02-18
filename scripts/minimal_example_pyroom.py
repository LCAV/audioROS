import pyroomacoustics as pra
import numpy as np
from math import ceil


fs = 32000  # sampling frequency [Hz]
duration_sec = 5  # duration of simulated audio signal [s]
mic_locs = np.c_[  # mics positions
    [6.3, 4.87, 1.2], [6.3, 4.93, 0.2],
]

source_pos = np.array(
    [  # source's consecutive positions [m]
        [2.5, 3.73, 1.76],
        [2.4, 3.73, 1.76],
        [2.3, 3.73, 1.76],
    ]
)

room_dim = np.array([9, 7.5, 3.5])  # room dimensions [m]
source_signal = 2.0 * (
    np.random.rand(int(ceil(fs * duration_sec))) - 0.5
)  # uniform white noise between -1 and 1
simulated_signals = []  # list storing signals at mics

for position in source_pos:
    pyroom = pra.ShoeBox(room_dim, fs=fs)
    pyroom.add_source(position, signal=source_signal)
    pyroom.add_microphone_array(mic_locs)
    pyroom.simulate()
    simulated_signals.append(pyroom.mic_array.signals)
