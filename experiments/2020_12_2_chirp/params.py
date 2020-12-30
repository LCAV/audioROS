global_params = {
    'source_type': 'buzzer-onboard',
    'fs_soundcard': 44100, # hz
    'n_meas_mics': 1, # number of measurement mics
}

MIN_FREQ = 0
MAX_FREQ = 16000 
THRUST = 50000
SOURCE = 'sweep_all'
D = 50

# repeat for many different distances
params_list = [
    {'min_freq':MIN_FREQ, 'max_freq':MAX_FREQ, 'distance':D, 'motors':0, 'snr': 0, 'props': 0, 'source':SOURCE, 'degree':0},
    {'min_freq':MIN_FREQ, 'max_freq':MAX_FREQ, 'distance':D, 'motors':THRUST, 'snr': 0, 'props': 0, 'source':SOURCE, 'degree':0},
    {'min_freq':MIN_FREQ, 'max_freq':MAX_FREQ, 'distance':D, 'motors':0, 'snr': 1, 'props': 0, 'source':SOURCE, 'degree':0},
    {'min_freq':MIN_FREQ, 'max_freq':MAX_FREQ, 'distance':D, 'motors':THRUST, 'snr': 1, 'props': 0, 'source':SOURCE, 'degree':0},
]
