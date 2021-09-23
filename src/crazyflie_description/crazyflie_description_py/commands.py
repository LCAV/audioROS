import numpy as np
from .parameters import SOUND_EFFECTS


TOTAL_DISTANCE_M = 0.5 # meters

VMIN_CMS = 3 # centimeter per second, minimum velocity


# Format of all commands:
# node, parameter, value, duration (seconds)
# duration calculation:
# for angles: angle_deg / 360 * 5
# for distances: distance_m * 5 (/0.2)
motor_command_lists = {
    "propsweep": [("/gateway", "buzzer_idx", 1, 0)]
    + [
        ("/gateway", "all", thrust, 1.0)
        for thrust in np.arange(45000, 61000, step=1000)
    ]
    + [("/gateway", "buzzer_idx", 0, 0), ("/gateway", "all", 0, 0)],
    "hover": [
        ("/gateway", "hover_height", 0.5, 0),
        ("/gateway", "move_forward", 0, 20,),  # stay in place, but wait for 20 seconds.
        ("/gateway", "land_velocity", 0.2, 3),
    ],
    "hover_sweep": [
        ("/gateway", "hover_height", 0.4, 0),
        ("/gateway", "buzzer_idx", 1, 0),
        ("", "", "", 20),  # time delay only
        ("/gateway", "land_velocity", 0.2, 0),
    ],
    "multi_sweep": [
        ("/gateway", "hover_height", 0.4, 0),  # start at 50
        ("/gateway", "buzzer_idx", 3, 0),
        ("", "", "", 1.5),  # time delay only
        ("/gateway", "buzzer_idx", 0, 0),
        ("/gateway", "move_distance", 0.2, 0),  # 30
        ("/gateway", "buzzer_idx", 3, 0),
        ("", "", "", 1.5),  # time delay only
        ("/gateway", "buzzer_idx", 0, 0),
        ("/gateway", "move_distance", 0.2, 0),  # 10 cm
        ("/gateway", "buzzer_idx", 3, 0),
        ("", "", "", 1.5),  # time delay only
        ("/gateway", "land_velocity", 0.2, 0),
    ],
    "linear_vmin": [
        ("/gateway", "hover_height", 0.5, 0),
        ("/gateway", "move_forward", VMIN_CMS * 1e-2, TOTAL_DISTANCE_M/(VMIN_CMS * 1e-2)),  # 50 cm
        ("/gateway", "land_velocity", 0.2, 3),
    ],
    "linear_buzzer_cont": [
        ("/gateway", "buzzer_idx", 1, 0),
        ("/gateway", "hover_height", 0.5, 0),
        ("/gateway", "move_forward", VMIN_CMS * 1e-2, TOTAL_DISTANCE_M/(VMIN_CMS * 1e-2)),  # 50 cm
	("/gateway", "land_velocity", 0.2, 3),
        ("/gateway", "buzzer_idx", 0, 0),
    ],
    "linear_sweep": [
        ("/gateway", "hover_height", 0.5, 0),
        ("/gateway", "buzzer_idx", 3000, 0),
        ("/gateway", "move_forward", 0.05, 8),  # 50 cm
        ("/gateway", "land_velocity", 0.2, 3),
    ],
    "linear_3000": [
        ("/gateway", "hover_height", 0.4, 0),
        ("/gateway", "buzzer_idx", 3000, 0),
        ("/gateway", "move_forward", 0.05, 10),  # 50 cm
        ("/gateway", "buzzer_idx", 0, 0),
        ("/gateway", "land_velocity", 0.2, 0),
    ],
    "linear_3000_fast": [
        ("/gateway", "hover_height", 0.4, 0),
        ("/gateway", "buzzer_idx", 3000, 0),
        ("/gateway", "move_forward", 0.10, 5),  # 50 cm
        ("/gateway", "buzzer_idx", 0, 0),
        ("/gateway", "land_velocity", 0.2, 0),
    ],
    "stop_motors": [("/gateway", "all", 0, 0)],
}

buzzer_command_lists = {
    key: [("/gateway", "buzzer_idx", value[0], 0)]
    for key, value in SOUND_EFFECTS.items()
}
buzzer_command_lists["stop_buzzer"] = [
    ("/gateway", "buzzer_idx", 0, 0),
]

assert (
    set(buzzer_command_lists.keys()).intersection(motor_command_lists.keys()) == set()
)

all_commands_lists = {**buzzer_command_lists, **motor_command_lists}
