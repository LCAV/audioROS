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
        ('/gateway', 'hover_height', 0.5, 0),
        ('/gateway', 'buzzer_effect', None, 0), # will be overwritten
        ('/gateway', 'move_forward', 0.05, 8), # 50 cm
        ('/gateway', 'land_velocity', 0.2, 3),
     ],
     # TODO(FD) solve this more elegantly
     'all5000': [
        ('/gateway', 'all', 5000, 0) 
     ],
     'all30000': [
        ('/gateway', 'all', 30000, 0) 
     ],
     'all43000': [
        ('/gateway', 'all', 43000, 0) 
     ],
     'all45000': [
        ('/gateway', 'all', 45000, 0) 
     ],
     'all50000': [
        ('/gateway', 'all', 50000, 0) 
     ],
     'all55000': [
        ('/gateway', 'all', 55000, 0) 
     ],
}

# buzzer commands from the different sweeps.
buzzer_dict = {
    key: [('/gateway', 'buzzer_effect', value[0], 0)] for key, value in SOUND_EFFECTS.items() if 'sweep' in key
}
buzzer_dict['stop'] =  [
    ('/gateway', 'buzzer_freq', 0, 0), # set frequency to 0 (for effect 12 only)
    ('/gateway', 'buzzer_effect', 0, 0), # turn effect off
]
# buzzer command for mono signals: first set to bypass, then set frequency. 
for key in SOUND_EFFECTS.keys():
    if 'mono' in key:
        f = int(key.strip('mono'))
        buzzer_dict[key] = [
            ('/gateway', 'buzzer_effect', 12, 0), # bypass
            ('/gateway', 'buzzer_freq', f, 0),
        ]
