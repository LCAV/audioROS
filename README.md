# audioROS

This repository contains the ROS2-powered stack to do audio processing for sound source localization on the Crazyflie drone 2.1. 

## Installation

The repository was built for Ubuntu 18.04.4 with ROS2 Eloquent and Python3.6.9. If you are testing with newer versions and run into problems please do not hesitate to get in touch.
We assume that you have a working python installation with `pip` ready to use.

Make sure to clone this repo including submodules by running
```
git clone --recurse-submodules https://github.com/LCAV/audioROS
```

After installing ROS, and in each new terminal, run
```
source /opt/ros/<distro>/setup.bash
```
where you replace `<distro>` with the installed distribution, e.g. eloquent.

To install all packages contained in this repo, including dependencies, run (from the root of this repository): 
```
sudo apt-get install python3-rosdep python3-colcon-common-extensions
sudo rosdep init
cp 19-custom.list /etc/ros/rosdep/sources.list.d/ #might need sudo here
rosdep update 
rosdep install --from-path src/ --rosdistro $ROS_DISTRO
colcon build --symlink-install
. install/local_setup.bash
```

## Contents    

The stack is separated into the following modules:

- `audio_interfaces` (C++): Custom ROS message definintions.
- `audio_bringup` (python): Contains pipeline for recording measurements (`doa_pipeline.py`) and some commonly used launch files.  
- `audio_stack` (python): Read and process audio signals.
- `audio_simulation` (python): Simulated audio using pyroomacoustics.  
- `audio_publisher` (python): Publish signals from file or computer's audio input stream. 
- `crazyflie_crtp` (python): Publish audio signals received over CRTP from Crazyflie drone.  
- `crazyflie_description` (python): Commonly used global parameters of Crazyflie drone.  
- `topic_plotter` (python): Create plots of the different topics. 
- `topic_writer` (python): Convert data from topics to different formats (e.g. csv format) and save. 


A diagram of the interactions of the different packages is shown below.

![Diagram showing package interactions](https://app.lucidchart.com/publicSegments/view/8da32e75-dd1a-45f2-a5a3-6a195968585d/image.png)

## Use cases

### Simulation 

- To simulate the drone doing source localization inside a room, you can run: 
  ```
  ros2 launch audio_bringup doa_simulated.launch.py
  ```

## Requirements (WIP)

List of requirements to run all nodes in this repo, continuously updated and will (hopefully) eventually be added to installation instructions.
- pyserial (for turntable handling)

## Credits

We took inspiration from [this](https://roboticsbackend.com/package-organization-for-a-ros-stack-best-practices/) helpful article for the structure of this repo. 
