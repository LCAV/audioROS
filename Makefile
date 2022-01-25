# run experiments along with the correct videos
glass_wall:
		ros2 bag play datasets/2021_11_23_demo/hover8 &
		sleep 2
		vlc --start-time=-3 --width=500 datasets/2021_11_23_demo/videos/glass-topdown.m4v &
		ros2 launch audio_bringup live_analysis.launch.py
normal_wall:
		ros2 bag play datasets/2021_11_23_demo/hover12 & 
		vlc --start-time=17 --width=500 datasets/2021_11_23_demo/videos/wall-full.mp4 &
		ros2 launch audio_bringup live_analysis.launch.py
abort: 
		ros2 service call /change_state_manual audio_interfaces/srv/StateMachine "state: 7"
stop: 
		ros2 service call /change_state_manual audio_interfaces/srv/StateMachine "state: 7"
stop_buzzer:
		ros2 service call /send_command_manual audio_interfaces/srv/CrazyflieCommands "{command_name: buzzer_idx, command_value: 0.0}"
land:
		ros2 service call /send_command_manual audio_interfaces/srv/CrazyflieCommands "{command_name: land_velocity, command_value: 0.2}"
move:
		ros2 service call /send_command_manual audio_interfaces/srv/CrazyflieCommands "{command_name: move_forward, command_value: 0.03}"
hover:
		ros2 service call /send_command_manual audio_interfaces/srv/CrazyflieCommands "{command_name: hover_height, command_value: 0.4}"
