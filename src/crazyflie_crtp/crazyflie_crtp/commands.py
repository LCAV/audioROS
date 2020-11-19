import numpy as np

# node, parameter, value, duration (seconds)
thrusts = np.linspace(1000, 50000, 50)
command_dict = {
     'sweep': [
        ('/gateway', 'all', thrust, 1) for thrust in thrusts
     ],
     'hover': [
        ('/gateway', 'hover_height', 0.5, 3),
        ('/gateway', 'turn_angle', 20, 5),
        ('/gateway', 'turn_angle', -20, 5),
        ('/gateway', 'land_velocity', 1, 3),
     ]
}

buzzer_dict = {
    'sweep': [
        ('/gateway', 'buzzer_effect', 11, 0),
    ],
    'mono4125': [
        ('/gateway', 'buzzer_effect', 12, 0), # bypass
        ('/gateway', 'buzzer_freq', 4125, 0),
    ],
    'stop': [
        ('/gateway', 'buzzer_effect', 0, 0), # off
    ]
}
