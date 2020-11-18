global_params = {
    'duration': 30, # seconds
    'source_type': 'buzzer',
    'fs_soundcard': 44100, # hz
    'n_meas_mics': 1, # number of measurement mics
    'freq_source': 4100, # hz
    'min_dB': -60,
    'max_dB': -10, 
}

THRUST = 43000
SOURCE = 'buzzer' # need to set this to None for buzzer 

# repeat for many different distances
params_list = []
for source in ['buzzer', None]:
    for d in np.linspace(0.1, 1.0, 10) * 1e-2:
        params_list += [
            {'distance':d, 'motors': THRUST, 'snr': 0, 'props': 0, 'source':source, 'degree':360},
        ]
