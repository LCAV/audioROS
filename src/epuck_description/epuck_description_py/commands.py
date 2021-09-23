from epuck_description_py.parameters import SOUND_EFFECTS

motor_command_lists = {
    "sweep_and_move": [("/gateway", "buzzer_idx", 1, 0.0),],
    "multi_sweep": [
        ("/gateway", "hover_height", 0.4, 0),  # start at 50
        ("/gateway", "buzzer_idx", 1, 0),
        ("", "", "", 4),  # time delay only
        ("/gateway", "buzzer_idx", 0, 0),
        ("/gateway", "move_distance", 0.2, 0),  # 30
        ("/gateway", "buzzer_idx", 1, 0),
        ("", "", "", 4),  # time delay only
        ("/gateway", "buzzer_idx", 0, 0),
        ("/gateway", "move_distance", 0.2, 0),  # 10 cm
        ("/gateway", "buzzer_idx", 1, 0),
        ("", "", "", 4),  # time delay only
        ("/gateway", "land_velocity", 0.2, 0),
    ],
    "linear_3000": [
        ("/gateway", "hover_height", 0.4, 0),
        ("/gateway", "buzzer_idx", 3000, 0),
        ("/gateway", "move_forward", 0.05, 10),  # 50 cm
        ("/gateway", "buzzer_idx", 0, 0),
        ("/gateway", "land_velocity", 0.2, 0),
    ],
    "stop_motors": [("/gateway", "stop", 0, 0)],
}

buzzer_command_lists = {
    key: [("/gateway", "buzzer_idx", value[0], 0)]
    for key, value in SOUND_EFFECTS.items()
}
buzzer_command_lists["stop_buzzer"] = [
    ("/gateway", "buzzer_idx", 0, 0),
]
all_commands_lists = {**buzzer_command_lists, **motor_command_lists}
