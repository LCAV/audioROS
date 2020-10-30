# TODO(FD) below parameters were reconstructed and might not correspond to the actual
# parameters with which the experiments were ran! At the time the pipeline was not as 
# automated yet. 

global_params = {
    'duration':30, # seconds
    'fs_soundcard':44100, # hz
    'n_meas_mics':1, # number of measurement mics
    'freq_source':800, # hz
    'min_dB':-30,
    'max_dB':0,
}

THRUST = 43000
DEGREE_LIST = [0, 20, 45] 
SOURCE_LIST = [True, False]

# calibration:
params_list = [
    {'motors': 0, 'snr': 0, 'props': 0, 'source':None, 'degree':0},
    {'motors': THRUST, 'snr': 0, 'props': 0, 'source':None, 'degree':0},
    {'motors': THRUST, 'snr': 0, 'props': 0, 'source':None, 'degree':0}
]

params_list += [
    {'motors': 0, 'snr': 0, 'props': 1, 'source':None, 'degree':0},
    {'motors': THRUST, 'snr': 0, 'props': 1, 'source':None, 'degree':0},
    {'motors': THRUST, 'snr': 1, 'props': 0, 'source':None, 'degree':0},
    {'motors': THRUST, 'snr': 1, 'props': 1, 'source':None, 'degree':0},
]
for degree in DEGREE_LIST:
    for source in SOURCE_LIST: 
        params_list += [
            {'motors': 0, 'snr': 1, 'props': 0, 'source':source, 'degree':degree},
            {'motors': 0, 'snr': 0, 'props': 0, 'source':source, 'degree':degree},
            {'motors': THRUST, 'snr': 0, 'props': 0, 'source':source, 'degree':degree},
            {'motors': THRUST, 'snr': 1, 'props': 0, 'source':source, 'degree':degree},
            {'motors': THRUST, 'snr': 0, 'props': 1, 'source':source, 'degree':degree},
            {'motors': THRUST, 'snr': 1, 'props': 1, 'source':source, 'degree':degree},
        ]
