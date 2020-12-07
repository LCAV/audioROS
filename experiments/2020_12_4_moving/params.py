global_params = {
    'source_type': 'buzzer-onboard',
    'fs_soundcard': 44100, # hz
    'n_meas_mics': 1, # number of measurement mics
    'duration': 10
}

MIN_FREQ = 1000
MAX_FREQ = 5000
THRUST = 43000

# calibration
source_list = ['mono3125', 'mono4156', 'mono8000']
d = 51
for source in source_list:
    params_list = [
        {'distance':10, 'motors': THRUST, 'snr': 1, 'props': 0, 'source':SOURCE, 'degree':360},
        {'distance':51, 'motors': THRUST, 'snr': 0, 'props': 0, 'source':SOURCE, 'degree':0},
        {'distance':0, 'motors': THRUST, 'snr': 0, 'props': 0, 'source':SOURCE, 'degree':360},
        {'distance':0, 'motors': THRUST, 'snr': 1, 'props': 0, 'source':None, 'degree':360},
        {'distance':-51, 'motors': THRUST, 'snr': 1, 'props': 0, 'source':None, 'degree':0},
        {'distance':10, 'motors': THRUST, 'snr': 1, 'props': 0, 'source':None, 'degree':360},
    ]
