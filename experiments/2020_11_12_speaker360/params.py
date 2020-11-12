global_params = {
    'duration': 30, # seconds
    'source_type': 'soundcard',
    'fs_soundcard': 44100, # hz
    'n_meas_mics': 1, # number of measurement mics
    'freq_source': 4100, # hz
    'min_dB': -60,
    'max_dB': -10, 
}

THRUST = 43000
SOURCE = 'mono' # need to set this to None for buzzer 

params_list = [
    # calibration:
    {'motors': 0, 'snr': 0, 'props': 0, 'source':None, 'degree':0},
    {'motors': THRUST, 'snr': 0, 'props': 0, 'source':None, 'degree':0},
    # static:
    {'motors': 0, 'snr': 0, 'props': 0, 'source':SOURCE, 'degree':0},
    {'motors': 0, 'snr': 0, 'props': 0, 'source':SOURCE, 'degree':90},
    {'motors': THRUST, 'snr': 0, 'props': 0, 'source':SOURCE, 'degree':0},
    {'motors': THRUST, 'snr': 0, 'props': 0, 'source':SOURCE, 'degree':90},
    # dynamic:
    {'motors': 0, 'snr': 0, 'props': 0, 'source':SOURCE, 'degree':45},
    {'motors': THRUST, 'snr': 0, 'props': 0, 'source':SOURCE, 'degree':45},
]
