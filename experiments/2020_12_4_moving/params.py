global_params = {
    'source_type': 'buzzer-onboard',
    'fs_soundcard': 44100, # hz
    'n_meas_mics': 1, # number of measurement mics
    'duration': 10 # will always be overwritten
}
THRUST = "all43000"

# calibration
params_list = []
for motors in [THRUST]:#[0, THRUST]:
    params_list += [
        {'distance':51, 'motors': motors, 'snr': 1, 'props': 0, 'source':'mono3125', 'degree':0, 
         'min_freq':1000, 'max_freq':5000},
        {'distance':-51, 'motors': motors, 'snr': 0, 'props': 0, 'source':'mono3125', 'degree':0},
        {'distance':51, 'motors': motors, 'snr': 1, 'props': 0, 'source':'mono4156', 'degree':0, 
         'min_freq':1000, 'max_freq':5000},
        {'distance':-51, 'motors': motors, 'snr': 0, 'props': 0, 'source':'mono4156', 'degree':0},
        {'distance':51, 'motors': motors, 'snr': 1, 'props': 0, 'source':'mono8000', 'degree':0, 
         'min_freq':5000, 'max_freq':9000},
        {'distance':-51, 'motors': motors, 'snr': 0, 'props': 0, 'source':'mono8000', 'degree':0},
    ]

params_list += [
    {'distance':51, 'motors': motors, 'snr': 1, 'props': 0, 'source':None, 'degree':0,
     'min_freq':100, 'max_freq':5000},
    {'distance':-51, 'motors': motors, 'snr': 1, 'props': 0, 'source':None, 'degree':0,
     'min_freq':5000, 'max_freq':9000},
]
