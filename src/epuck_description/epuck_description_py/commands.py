from epuck_description_py.parameters import SOUND_EFFECTS

motor_command_lists = {
    "sweep_and_move": [
        ("/gateway", "buzzer_idx", 1, 10.0),
        ("/gateway", "move_forward", 1, 2.0),
    ],
    "stop_motors": [("/gateway", "stop", 1, 0)],
}

buzzer_command_lists = {
    key: [("/gateway", "buzzer_idx", value[0], 0)]
    for key, value in SOUND_EFFECTS.items()
}
buzzer_command_lists["stop_buzzer"] = [
    ("/gateway", "buzzer_idx", 0, 0),
]
all_commands_lists = {**buzzer_command_lists, **motor_command_lists}
