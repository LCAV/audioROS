import numpy as np
from .parameters import SWEEPS

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

# buzzer commands from the different sweeps.
buzzer_dict = {
    key: [('/gateway', 'buzzer_effect', value[0], 0)] for key, value in SWEEPS.items() if 'sweep' in key
}
buzzer_dict.update({
    'mono4125': [
        ('/gateway', 'buzzer_effect', 12, 0), # bypass
        ('/gateway', 'buzzer_freq', 4125, 0),
    ],
    'mono3500': [
        ('/gateway', 'buzzer_effect', 12, 0), # bypass
        ('/gateway', 'buzzer_freq', 3500, 0),
    ],
    'stop': [
        ('/gateway', 'buzzer_effect', 0, 0), # off
    ]
})
