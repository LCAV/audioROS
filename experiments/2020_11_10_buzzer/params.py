global_params = {
    'duration': 30, # seconds
    'fs_soundcard': 44100, # hz
    'n_meas_mics': 1, # number of measurement mics
    'freq_source': 828, # hz
    'min_dB': -60,
    'max_dB': -10, 
}

THRUST = 43000

# calibration:
params_list = [
    {'motors': 0, 'snr': 0, 'props': 0, 'source':None, 'degree':0},
    {'motors': 0, 'snr': 0, 'props': 0, 'source':None, 'degree':90},
    {'motors': THRUST, 'snr': 0, 'props': 0, 'source':None, 'degree':0},
    {'motors': THRUST, 'snr': 0, 'props': 0, 'source':None, 'degree':90},
    {'motors': 0, 'snr': 0, 'props': 0, 'source':None, 'degree':45},
    {'motors': THRUST, 'snr': 0, 'props': 0, 'source':None, 'degree':45},
]
params_list = [
    {'motors': THRUST, 'snr': 0, 'props': 0, 'source':None, 'degree':0},
]
