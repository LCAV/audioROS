global_params = {
    'source_type': 'buzzer-onboard',
    'fs_soundcard': 44100, # hz
    'n_meas_mics': 0, # number of measurement mics
    'duration': 10 # will always be overwritten
}

THRUST = "all43000"
SOURCE_LIST = ['sweep', 'sweep_buzzer']
DISTANCE_LIST = [None]
DEGREE_LIST = [0]
MOTORS_LIST = [0, THRUST]
MIN_FREQ = 1000
MAX_FREQ = 5000

# calibration
params_list = [
   {'motors':0, 'snr':0, 'props':0, 'source':'sweep', 'min_freq':MIN_FREQ, 'max_freq':MAX_FREQ}, 
   {'motors':0, 'snr':1, 'props':1, 'source':'sweep', 'min_freq':MIN_FREQ, 'max_freq':MAX_FREQ}, 
   {'motors':0, 'snr':1, 'props':1, 'source':'sweep_buzzer'}, 
   {'motors':THRUST, 'snr':0, 'props':0, 'source':'sweep', 'min_freq':MIN_FREQ, 'max_freq':MAX_FREQ}, 
   {'motors':THRUST, 'snr':1, 'props':1, 'source':'sweep', 'min_freq':MIN_FREQ, 'max_freq':MAX_FREQ}, 
]
