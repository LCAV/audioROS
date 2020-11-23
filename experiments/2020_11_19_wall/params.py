global_params = {
    'duration': 38, # seconds, 
    'source_type': 'buzzer-onboard',
    'fs_soundcard': 44100, # hz
    'n_meas_mics': 1, # number of measurement mics
    'freq_source': 4125, # hz, for mono only
    'min_dB': -60,
    'max_dB': -10, 
}

THRUST = 43000
#MIN_FREQ = 3000 # to be fixed
#MAX_FREQ = 15000
MIN_FREQ = 100 # to be fixed
MAX_FREQ = 4400
MONO = 'mono4125'

# repeat for many different distances
params_list = []
for d in [0, 10, 20, 30, 40, 50, 60, 70, 80, 90]:
    for motors in [0, THRUST]:
        params_list += [
            {'min_freq':MIN_FREQ, 'max_freq':MAX_FREQ, 'distance':d, 'motors': motors, 'snr': 0, 'props': 0, 'source':MONO, 'degree':360},
            {'min_freq':MIN_FREQ, 'max_freq':MAX_FREQ, 'distance':d, 'motors': motors, 'snr': 0, 'props': 0, 'source':None, 'degree':360},
            {'min_freq':MIN_FREQ, 'max_freq':MAX_FREQ, 'distance':d, 'motors': motors, 'snr': 0, 'props': 0, 'source':'sweep', 'degree':0},
            {'min_freq':MIN_FREQ, 'max_freq':MAX_FREQ, 'distance':d, 'motors': motors, 'snr': 0, 'props': 0, 'source':'sweep', 'degree':27},
            {'min_freq':MIN_FREQ, 'max_freq':MAX_FREQ, 'distance':d, 'motors': motors, 'snr': 0, 'props': 0, 'source':'sweep', 'degree':54},
            {'min_freq':MIN_FREQ, 'max_freq':MAX_FREQ, 'distance':d, 'motors': motors, 'snr': 0, 'props': 0, 'source':'sweep', 'degree':81}
        ]
