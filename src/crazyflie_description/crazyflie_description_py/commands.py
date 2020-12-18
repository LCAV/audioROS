import numpy as np
from .parameters import SOUND_EFFECTS

# node, parameter, value, duration (seconds)
# 
# duration calculation: 
# for angles: angle_deg / 360 * 5
# for distances: distance_m * 5 (/0.2)
thrusts = np.linspace(1000, 50000, 50)
command_dict = {
     'sweep': [
        ('/gateway', 'all', thrust, 1) for thrust in thrusts
     ],
     'hover': [
        ('/gateway', 'hover_height', 0.5, 3),
        ('/gateway', '' , '' , 2),
        ('/gateway', 'turn_angle', 360, 5), # rate is 5
        ('/gateway', '' , '' , 2),
        ('/gateway', 'move_distance', 0.3, 2),
        ('/gateway', '' , '' , 2),
        ('/gateway', 'move_distance', -0.3, 2),
        ('/gateway', '' , '' , 2),
        ('/gateway', 'land_velocity', 0.2, 3),
     ],
     'linear':[
        ('/gateway', 'hover_height', 0.5, 3),
        ('/gateway', 'move_distance', 0.5, 2), # 50 cm
        ('/gateway', 'land_velocity', 0.2, 3),
     ],

     'all43000': [
        ('/gateway', 'all', 43000, 1) # 165 is duration of the 50cm move
     ],
}

# buzzer commands from the different sweeps.
buzzer_dict = {
    key: [('/gateway', 'buzzer_effect', value[0], 0)] for key, value in SOUND_EFFECTS.items() if 'sweep' in key
}

# buzzer command for mono signals: first set to bypass, then set frequency. 
for key in SOUND_EFFECTS.keys():
    if 'mono' in key:
        f = int(key.strip('mono'))
        buzzer_dict[key] = [
            ('/gateway', 'buzzer_effect', 12, 0), # bypass
            ('/gateway', 'buzzer_freq', f, 0),
        ]
buzzer_dict['stop'] =  [
    ('/gateway', 'buzzer_effect', 0, 0), # off
]
