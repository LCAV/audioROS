global_params = {
    'source_type': 'buzzer-onboard',
    'fs_soundcard': 44100, # hz
    'n_meas_mics': 1, # number of measurement mics
    'duration': 10
}

MIN_FREQ = 1000
MAX_FREQ = 5000
THRUST = 50000

# calibration
d = 0
params_list = [
    {'min_freq':MIN_FREQ, 'max_freq':MAX_FREQ, 'distance':d, 'motors': THRUST, 'snr': 0, 'props': 0, 'source':None, 'degree':0},
    {'min_freq':MIN_FREQ, 'max_freq':MAX_FREQ, 'distance':d, 'motors': 0, 'snr': 0, 'props': 0, 'source':None, 'degree':0},
]

for d in range(50):
    for degree in [0, 27, 54, 81]:
        params_list += [
            {'distance':d, 'motors': THRUST, 'snr': 0, 'props': 0, 'source':'sweep', 'degree':degree},
            {'distance':d, 'motors': 0, 'snr': 0, 'props': 0, 'source':'sweep', 'degree':degree},
            {'distance':d, 'motors': THRUST, 'snr': 0, 'props': 0, 'source':'sweep_hard', 'degree':degree},
            {'distance':d, 'motors': 0, 'snr': 0, 'props': 0, 'source':'sweep_hard', 'degree':degree},
        ]
