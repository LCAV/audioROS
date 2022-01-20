# run experiments along with the correct videos
glass_wall:
		ros2 bag play datasets/2021_11_23_demo/hover8 &
		#sleep 2
		#vlc --start-time=-3 --width=500 datasets/2021_11_23_demo/videos/glass-topdown.m4v &
		ros2 launch audio_bringup live_analysis.launch.py
normal_wall:
		ros2 bag play datasets/2021_11_23_demo/hover12 & 
		vlc --start-time=17 --width=500 datasets/2021_11_23_demo/videos/wall-full.mp4 &
		ros2 launch audio_bringup live_analysis.launch.py
