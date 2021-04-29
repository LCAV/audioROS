import numpy as np
from .parameters import SOUND_EFFECTS

# Format of all commands: 
# node, parameter, value, duration (seconds)

# duration calculation: 
# for angles: angle_deg / 360 * 5
# for distances: distance_m * 5 (/0.2)
motor_command_lists = {
     'hover': [
        ('/gateway', 'hover_height', 0.2, 0),
        ('/gateway', 'move_forward', 0, 20), # stay in place, but wait for 20 seconds.
        ('/gateway', 'land_velocity', 0.2, 3),
     ],
     'hover_sweep_short': [
        ('/gateway', 'hover_height', 0.4, 0),
        #('/gateway', 'move_distance', 0.01, 0), # 10 cm
        ('/gateway', 'buzzer_effect', 17, 0),
        ('', '', '', 20), # time delay only
        ('/gateway', 'land_velocity', 0.2, 0),
     ],
     'linear':[
        ('/gateway', 'hover_height', 0.5, 0),
        ('/gateway', 'move_forward', 0.05, 8), # 50 cm
        ('/gateway', 'land_velocity', 0.2, 3),
     ],
     'linear_sweep_slow':[
        ('/gateway', 'hover_height', 0.5, 0),
        ('/gateway', 'buzzer_effect', 21, 0), 
        ('/gateway', 'move_forward', 0.05, 8), # 50 cm
        ('/gateway', 'land_velocity', 0.2, 3),
     ],
     'linear_sweep_fast':[
        ('/gateway', 'hover_height', 0.5, 0),
        ('/gateway', 'buzzer_effect', 22, 0), 
        ('/gateway', 'move_forward', 0.05, 8), # 50 cm
        ('/gateway', 'land_velocity', 0.2, 3),
     ],
     'stop_motors': [
        ('/gateway', 'all', 0, 0)
     ]
}

buzzer_command_lists = {
    key: [('/gateway', 'buzzer_effect', value[0], 0)] for key, value in SOUND_EFFECTS.items()
}
buzzer_command_lists['stop_buzzer'] =  [
    ('/gateway', 'buzzer_freq', 0, 0), # set frequency to 0 (for effect 12 only)
    ('/gateway', 'buzzer_effect', 0, 0), # turn effect off
]

assert set(buzzer_command_lists.keys()).intersection(motor_command_lists.keys()) == set()

all_commands_lists = {
    **buzzer_command_lists,
    **motor_command_lists
}
