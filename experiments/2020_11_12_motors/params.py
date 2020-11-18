global_params = {
    'duration': 30, # seconds
    'source_type': 'buzzer',
    'fs_soundcard': 44100, # hz
    'n_meas_mics': 0, # number of measurement mics
    'freq_source': 4100, # hz
    'min_dB': -60,
    'max_dB': -10, 
}

params_list = [
    {'motors': 'sweep', 'snr': 0, 'props': 0, 'source':None, 'degree':0},
    {'motors': 'hover', 'snr': 0, 'props': 0, 'source':None, 'degree':0},
    {'motors': 'hover', 'snr': 0, 'props': 0, 'source':'buzzer', 'degree':0},
]
