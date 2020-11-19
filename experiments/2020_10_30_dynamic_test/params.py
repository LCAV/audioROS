
global_params = {
    'duration': 10, # seconds
    'fs_soundcard': 44100, # hz
    'n_meas_mics': 1, # number of measurement mics
    'freq_source': 800, # hz
    'min_dB': -10,
    'max_dB': 20, 
}

THRUST = 43000
SOURCE_LIST = ['mono', 'mono_linear']
DEGREE = 0

# calibration:
params_list = [
    {'motors': 0, 'snr': 0, 'props': 0, 'source':None, 'degree':DEGREE},
    {'motors': THRUST, 'snr': 0, 'props': 0, 'source':None, 'degree':DEGREE},
]

for source in SOURCE_LIST: 
    params_list += [
        {'motors': 0, 'snr': 0, 'props': 0, 'source':source, 'degree':DEGREE},
        #{'motors': 0, 'snr': 1, 'props': 0, 'source':source, 'degree':DEGREE},
        {'motors': THRUST, 'snr': 0, 'props': 0, 'source':source, 'degree':DEGREE},
        #{'motors': THRUST, 'snr': 1, 'props': 0, 'source':source, 'degree':DEGREE},
        #{'motors': THRUST, 'snr': 0, 'props': 1, 'source':source, 'degree':DEGREE},
        #{'motors': THRUST, 'snr': 1, 'props': 1, 'source':source, 'degree':DEGREE},
    ]
