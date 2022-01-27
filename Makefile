# run experiments along with the correct videos
glass_wall:
		ros2 launch audio_bringup live_analysis.launch.py
normal_wall:
		ros2 launch audio_bringup live_analysis.launch.py

# shortcuts for long service calls
abort: 
		ros2 service call /change_state_manual audio_interfaces/srv/StateMachine "state: 7"
stop: abort
avoid:
		ros2 service call /change_state_manual audio_interfaces/srv/StateMachine "state: 5"
stop_buzzer:
		ros2 service call /send_command_manual audio_interfaces/srv/CrazyflieCommands "{command_name: buzzer_idx, command_value: 0.0}"
land:
		ros2 service call /send_command_manual audio_interfaces/srv/CrazyflieCommands "{command_name: land_velocity, command_value: 0.2}"
move:
		ros2 service call /send_command_manual audio_interfaces/srv/CrazyflieCommands "{command_name: move_forward, command_value: 0.03}"
hover:
		ros2 service call /send_command_manual audio_interfaces/srv/CrazyflieCommands "{command_name: hover_height, command_value: 0.4}"

