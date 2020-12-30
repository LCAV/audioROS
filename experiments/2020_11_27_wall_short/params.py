global_params = {
    'fs_soundcard': 44100, # hz
    'source_type': 'buzzer-onboard',
    'n_meas_mics': 1, # number of measurement mics
}

THRUST = 43000
sources = ['sweep_low', 'sweep_high']

params_list = []
for d in range(50):
    for source in sources:
        for motors in [0, THRUST]:
            params_list += [
                {'distance':d, 'motors':motors, 'snr': 0, 'props': 0, 'source':source, 'degree':0},
            ]
