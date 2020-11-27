global_params = {
    'duration': 38, # seconds, 
    'source_type': 'buzzer-onboard',
    'fs_soundcard': 44100, # hz
    'n_meas_mics': 1, # number of measurement mics
    'freq_source': 4125, # hz, for mono only
    'min_dB': -60,
    'max_dB': -10, 
}

MIN_FREQ = 1000
MAX_FREQ = 5000
THRUST = 43000

# repeat for many different distances
params_list = []
for d in range(50):
    params_list += [
        {'min_freq':MIN_FREQ, 'max_freq':MAX_FREQ, 'distance':d, 'motors': THRUST, 'snr': 0, 'props': 0, 'source':'sweep', 'degree':0},
    ]
