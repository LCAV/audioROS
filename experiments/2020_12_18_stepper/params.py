global_params = {
    'source_type': 'buzzer-onboard',
    'fs_soundcard': 44100, # hz
    'n_meas_mics': 1, # number of measurement mics
    'duration': 26
}

MOTORS = "all45000" 
SNR = 2 
FREQS = [1750, 2375, 3125, 3875] 
MIN_FREQ = 1000
MAX_FREQ = 5000

SOURCE_LIST = ['sweep_slow', 'sweep_fast', None]
SOURCE_LIST += [f'mono{f}' for f in FREQS]
DISTANCE_LIST = [51, -51]
DEGREE_LIST = [0]
PROPS_LIST = [0, 1]
MOTORS_LIST = [MOTORS]
SNR_LIST = [SNR]

params_list = []

for f in FREQS:
    params_list += [
        {'motors':MOTORS, 'distance':51, 'snr':SNR, 'props':1, 'source':f'mono{f}', 'min_freq':MIN_FREQ, 'max_freq':MAX_FREQ}, 
        {'motors':0, 'distance':-51, 'snr':SNR, 'props':1, 'source':f'mono{f}', 'min_freq':MIN_FREQ, 'max_freq':MAX_FREQ}, 
    ]

params_list += [
   {'motors':MOTORS, 'distance':51, 'snr':SNR, 'props':1, 'source':'sweep_fast', 'min_freq':MIN_FREQ, 'max_freq':MAX_FREQ}, 
   {'motors':0, 'distance':-51, 'snr':SNR, 'props':1, 'source':'sweep_fast', 'min_freq':MIN_FREQ, 'max_freq':MAX_FREQ}, 
   {'motors':MOTORS, 'distance':51, 'snr':SNR, 'props':0, 'source':None, 'min_freq':100, 'max_freq':MAX_FREQ}, 
]
