global_params = {
    'source_type': 'buzzer-onboard',
    'fs_soundcard': 44100, # hz
    'n_meas_mics': 1, # number of measurement mics
    'duration': 10 # will always be overwritten
}

THRUST = "all43000"
SOURCE_LIST = ['mono3125', 'mono4156', 'mono8000', None]
DISTANCE_LIST = [-51, 51]
DEGREE_LIST = [0]
MOTORS_LIST = [0, THRUST]

# calibration
params_list = []

for source in SOURCE_LIST[:-1]:
    params_list += [
        {'distance':51, 'motors': THRUST, 'snr': 0, 'props': 0, 'source':source, 'degree':0}, 
        {'distance':-51, 'motors': 0, 'snr': 0, 'props': 0, 'source':source, 'degree':0},
    ]

params_list += [
    {'distance':51, 'motors': THRUST, 'snr': 1, 'props': 0, 'source':None, 'degree':0,
     'min_freq':100, 'max_freq':5000},
    {'distance':-51, 'motors': THRUST, 'snr': 1, 'props': 0, 'source':None, 'degree':0,
     'min_freq':100, 'max_freq':5000},
]
