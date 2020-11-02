THRUST = 43000
DEGREE_LIST = [0, 20, 45]
SOURCE_LIST = ['mono_linear', 'random_linear']

# calibration:
params_list = [
    {'motors': 0, 'snr': 0, 'props': 0, 'source':None, 'degree':0},
    {'motors': 0, 'snr': 0, 'props': 1, 'source':None, 'degree':0},
    {'motors': THRUST, 'snr': 0, 'props': 0, 'source':None, 'degree':0},
    {'motors': THRUST, 'snr': 0, 'props': 1, 'source':None, 'degree':0},
    {'motors': THRUST, 'snr': 1, 'props': 0, 'source':None, 'degree':0},
    {'motors': THRUST, 'snr': 1, 'props': 1, 'source':None, 'degree':0},
    {'motors': 43000, 'snr': 0, 'props': 0, 'source':None, 'degree':0},
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
