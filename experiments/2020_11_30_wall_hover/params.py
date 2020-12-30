global_params = {
    'source_type': 'buzzer-onboard',
    'fs_soundcard': 44100, # hz
    'n_meas_mics': 1, # number of measurement mics
}

MIN_FREQ = 200
MAX_FREQ = 800
MOTORS = 'hover'

# repeat for many different distances
params_list = [
    {'min_freq':MIN_FREQ, 'max_freq':MAX_FREQ, 'motors':MOTORS, 'snr':True, 'props':False, 'source':None, 'degree':0},
]
