#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
parameters.py: Parameters specific to the Crazyflie drone.
"""

HEIGHT_MIC_ARRAY = 0.0 # height of mic array with respect to drone center (in meters)
HEIGHT_BUZZER = 0.01 # height of buzzer with resepect to drone center (in meteers) 
MIC_D = 0.108  # distance between mics (meters)
MIC_POSITIONS_UNIT = [[-1, -1], [1, -1], [-1, 1], [1, 1]]
MIC_POSITIONS = [[MIC_D / 2 * m for m in mics] for mics in MIC_POSITIONS_UNIT] # relative mic positions
BUZZER_POSITION = [[0.0, 0.0]] # relative buzzer position (in meters)
N_MICS = 4 # number of mics
FS = 32000 # sampling frequency [Hz]
N_BUFFER = 2048 # number of samples in audio buffer
FFTSIZE = 32 # number of frequency bins that are sent. 

TUKEY_ALPHA = 0.2 # alpha parameter for tukey window

MONO_FREQS = [3500, 4125, 1750, 2375, 3125, 3875] 

# name: (effect_number, [min_freq_Hz, max_freq_Hz], duration_sec)
SOUND_EFFECTS = {
    'sweep':      (15, [1000, 5000], 38.0), # 38s is duration of sweep
    'sweep_all':  (18, [0,   16000],  513), # 513 is duration of sweep
    'sweep_hard': (19, [1,       2], 12.0), # 12s is duration of sweep
    'sweep_buzzer':  (20, [0,   16000],  185), # 185 is duration of sweep
    'sweep_slow':   (21,  [1000, 5000], 0), # 1 will be overwritten
    'sweep_fast':   (22,  [1000, 5000], 0), # 1 will be overwritten
    'mono3125':   (12,  [1,       2], 0), # 1, 2 will lead to hard-coded bins
    'mono4156':   (12,  [1,       2], 0), 
    'mono8000':   (12,  [1,       2], 0), 
}
for f in MONO_FREQS:
    SOUND_EFFECTS[f'mono{f}'] = (12,  [1000, 5000], 0)
