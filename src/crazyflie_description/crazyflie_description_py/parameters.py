#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
parameters.py: Parameters specific to the Crazyflie drone.
"""
MIC_D = 0.0607  # distance of mics from centre
MIC_POSITIONS_UNIT = [
    [0, 1],
    [-1, 0],
    [0, -1],
    [1, 0],
]  # relative mic positions, normalized.
MIC_POSITIONS = [
    [MIC_D * m for m in mics] for mics in MIC_POSITIONS_UNIT
]  # relative mic positions (meters)

HEIGHT_MIC_ARRAY = 0.0  # height of mic array with respect to drone center (in meters)

BUZZER_POSITION = [[0.0, 0.0]]  # relative buzzer position (in meters)
HEIGHT_BUZZER = 0.01  # height of buzzer with resepect to drone center (in meters)

N_MICS = 4  # number of mics
FS = 64000  # sampling frequency [Hz]
N_BUFFER = 1048  # number of samples in audio buffer
FFTSIZE = 32  # number of frequency bins that are sent.

# name: (effect_number, [min_freq_Hz, max_freq_Hz], duration_sec)
SOUND_EFFECTS = {
    "sweep": (15, [1000, 5000], 38.0),
    "sweep_high": (16, [2000, 6000], 38.0),
    "sweep_short": (17, [3000, 5000], 20.0),
    "sweep_all": (18, [0, 16000], 513),
    "sweep_buzzer": (20, [0, 16000], 185),
    "sweep_slow": (21, [1000, 5000], 0),  # 0 will be overwritten
    "sweep_fast": (22, [1000, 5000], 0),  # 0 will be overwritten
    "sweep_new": (3, [2000, 6000], 6),
}
