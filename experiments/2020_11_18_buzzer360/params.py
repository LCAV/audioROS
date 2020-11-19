global_params = {
    'duration': 30, # seconds
    'source_type': 'buzzer',
    'fs_soundcard': 44100, # hz
    'n_meas_mics': 1, # number of measurement mics
    'freq_source': 4100, # hz
    'min_dB': -60,
    'max_dB': -10, 
}

THRUST = 43000

params_list = [
    {'motors': 0, 'snr': 0, 'props': 0, 'source':'buzzer', 'degree':360},
    {'motors': THRUST, 'snr': 0, 'props': 0, 'source':'buzzer', 'degree':360},
]
