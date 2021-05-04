#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
parameters.py: Parameters specific to the epuck 2.
"""

MIC_D = 0.059  # distance between mics (meters)
MIC_POSITIONS = [[-0.033, -0.003], [0.026, -0.002], [-0.003, 0.029], [-0.003, -0.033]] # relative mic positions (meters)
HEIGHT_MIC_ARRAY = 0.0 # height of mic array with respect to epuck center (in meters)

BUZZER_POSITION = [[0.0, 0.0]] # relative buzzer position (in meters)
HEIGHT_BUZZER = 0.01 # height of buzzer with respect to epuck center (in meters)

N_MICS = 4 # number of mics
FS = 16000 # sampling frequency [Hz]
N_BUFFER = 2048 # number of samples in audio buffer (160 per microphones after demodulation)
FFTSIZE = 32 # number of frequency bins that are sent. 

# name: (effect_number, [min_freq_Hz, max_freq_Hz], duration_sec)

