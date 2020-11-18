global_params = {
    'duration': 20, # seconds
    'source_type': 'soundcard',
    'fs_soundcard': 44100, # hz
    'n_meas_mics': 1, # number of measurement mics
    'freq_source': 4125, # hz
    'min_dB': -60,
    'max_dB': -10, 
}

THRUST = 43000
SOURCE = 'mono_linear' # need to set this to None for buzzer 

params_list = [
    {'min_freq': 3000, 'max_freq': 15000, 'motors': 0, 'snr': 0, 'props': 0, 'source':SOURCE, 'degree':360}, 
    {'min_freq': 3000, 'max_freq': 15000, 'motors': THRUST, 'snr': 0, 'props': 0, 'source':SOURCE, 'degree':360}, 
    {'min_freq': 3000, 'max_freq': 15000, 'motors': 0, 'snr': 0, 'props': 0, 'source':SOURCE, 'degree':0}, 
    {'min_freq': 3000, 'max_freq': 15000, 'motors': THRUST, 'snr': 0, 'props': 0, 'source':SOURCE, 'degree':0},
    {'min_freq': 3000, 'max_freq': 15000, 'motors': 0, 'snr': 0, 'props': 0, 'source':SOURCE, 'degree':90}, 
    {'min_freq': 3000, 'max_freq': 15000, 'motors': THRUST, 'snr': 0, 'props': 0, 'source':SOURCE, 'degree':90}
]
