#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
parameters.py: Parameters specific to the epuck 2.
"""

MIC_D = 0.059  # distance between mics (meters)
MIC_POSITIONS_NONCENT = [
    [-0.033, -0.003],
    [0.026, -0.002],
    [-0.003, 0.029],
    [-0.003, -0.033],
]  # relative mic positions (meters)
B = [-0.022, 0.015]  # relative buzzer position (in meters)
MIC_POSITIONS = [[m[0] - B[0], m[1] - B[1]] for m in MIC_POSITIONS_NONCENT]
BUZZER_POSITION = [[0, 0]]

HEIGHT_MIC_ARRAY = 0.0  # height of mic array with respect to epuck center (in meters)

HEIGHT_BUZZER = 0.01  # height of buzzer with respect to epuck center (in meters)

WHEEL_DIAMETER = 0.041  # wheel radius in meters

N_MICS = 4  # number of mics
FS = 16000  # sampling frequency [Hz]
N_BUFFER = (
    1024  # number of samples in audio buffer (160 per microphones after demodulation)
)
FFTSIZE = 32  # number of frequency bins that are sent.

# name: (effect_number, [min_freq_Hz, max_freq_Hz], duration_sec)
SWEEP_LENGTH = 32
SOUND_EFFECTS = {
    "sweep_epuck": (1, [3000, 5000], 6),
}
