global_params = {
    'fs_soundcard': 44100, # hz
    'source_type': 'buzzer-onboard',
    'n_meas_mics': 1, # number of measurement mics
}

THRUST = 43000

params_list = []
for d in range(30):
    params_list += [
        {'distance':d, 'motors':0, 'snr': 0, 'props': 0, 'source':'sweep', 'degree':0},
        {'distance':d, 'motors':0, 'snr': 0, 'props': 0, 'source':'sweep', 'degree':27},
        {'distance':d, 'motors':0, 'snr': 0, 'props': 0, 'source':'sweep', 'degree':54},
        {'distance':d, 'motors':0, 'snr': 0, 'props': 0, 'source':'sweep', 'degree':81},
        {'distance':d, 'motors':THRUST, 'snr': 0, 'props': 0, 'source':'sweep', 'degree':0},
        {'distance':d, 'motors':0, 'snr': 0, 'props': 0, 'source':'mono3500', 'degree':360},
        {'distance':d, 'motors':THRUST, 'snr': 0, 'props': 0, 'source':'mono3500', 'degree':360},
    ]
