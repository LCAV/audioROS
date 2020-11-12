import numpy as np
# node, parameter, value, duration (seconds)
thrusts = np.arange(5000, 50000, step=50000)
motor_sweep_commands = [
  ('/gateway', 'all', 5000, 5),
  ('/gateway', 'all', 10000, 5),
  ('/gateway', 'all', 20000, 5),
]

motor_hover_commands = [
  ('/gateway', 'hover_height', 0.5, 3),
  ('/gateway', 'turn_angle', 20, 5),
  ('/gateway', 'turn_angle', -20, 5),
  ('/gateway', 'land_velocity', 1, 3),
]

global_params = {
    'duration': 30, # seconds
    'source_type': 'buzzer',
    'fs_soundcard': 44100, # hz
    'n_meas_mics': 0, # number of measurement mics
    'freq_source': 4100, # hz
    'min_dB': -60,
    'max_dB': -10, 
    'sweep': motor_sweep_commands,
    'hover': motor_hover_commands
}

MOTORS = 'hover'
SOURCE = 'buzzer'

params_list = [
    # calibration:
    {'motors': MOTORS, 'snr': 0, 'props': 0, 'source':None, 'degree':0},
    {'motors': MOTORS, 'snr': 0, 'props': 0, 'source':SOURCE, 'degree':0},
]
