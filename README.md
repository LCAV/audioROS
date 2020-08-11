# crazyflie-audio-ROS

This repository contains the ROS2-powered stack to do audio processing for sound source localization on the Crazyflie drone 2.1. 

## Installation

Make sure to clone this repo including submodules by running
```
git clone --recurse-submodules https://github.com/lcav/crazyflie-audio-ROS
```

To install all packages contained in this repo, run (from the root of this repository): 

```
ln -s 19-custom.list /etc/ros/rosdep/sources.list.d/
rosdep update 
rosdep install --from-path src/
colcon build --symlink-install
. install/local_setup.bash
```

## Contents    

The stack is separated into the following modules:

- `audio_stack` (python): 
- `audio_interfaces` (C++): Custom ROS message definintions. Can be extended with services and/or actions in the future. 

To publish autocorrelation data, obtained from an audio file, a sounddevice or the CRTP packages sent by the crazyflie, 
adjust the file `src/audio_stack/audio_stack/publisher.py and run:

```
ros2 run audio_stack publisher
```

To process the data and plot the resulting DOA spectrum, run:
```
ros2 run audio_stack processor
```

## Credits

We followed the recommendations from [this](https://roboticsbackend.com/package-organization-for-a-ros-stack-best-practices/) article for the structure of this repo. 
