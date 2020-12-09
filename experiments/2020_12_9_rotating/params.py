global_params = {
    'source_type': 'buzzer-onboard',
    'fs_soundcard': 44100, # hz
    'n_meas_mics': 1, # number of measurement mics
    'duration': 10 # will always be overwritten
}

THRUST = "all43000"
SOURCE_LIST = ['sweep']
DISTANCE_LIST = [5, 10, 20, 30, 40, 50]
DEGREE_LIST = [90 * i for i in range(4)]
MOTORS_LIST = [0, THRUST]

source = SOURCE_LIST[0]

# calibration
params_list = []
for d in DISTANCE_LIST:
    params_list += [
        {'distance':d, 'motors': THRUST, 'snr': 0, 'props': 0, 'source':source, 'degree':0}, 
    ]
    for deg in DEGREE_LIST: 
        params_list += [
            {'distance':d, 'motors': 0, 'snr': 0, 'props': 0, 'source':source, 'degree':deg}, 
        ]
