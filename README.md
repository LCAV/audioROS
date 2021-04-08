# audioROS

This repository contains the ROS2-powered stack to do audio processing for sound source localization on the Crazyflie drone 2.1. 

## Installation

The repository was built for Ubuntu 18.04.4 with ROS2 Eloquent and Python3.6.9. If you are testing with newer versions and run into problems please do not hesitate to get in touch.
We assume that you have a working python installation with `pip` ready to use.
If you do not wish to install ubuntu as a dual boot, you can use the EPFL guide on VM's to install ubuntu 20.04(windows/mac) : 
https://enacit.epfl.ch/virtualisation/virtualbox.shtml

Make sure to clone this repo including submodules by running
```
git clone --recurse-submodules https://github.com/LCAV/audioROS
```
If previous function does not run because there are private submodules that do not need to be installed, you can use 
```
git -c submodule."<submodule_name>".update=none clone --recurse-submodules https://github.com/LCAV/audioROS
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
- `audio_bringup` (python): Contains pipeline for recording measurements (`measurement_pipeline.py`) and some commonly used launch files.  
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

- To simulate the drone doing sound source localization inside a room, run: 
  ```
  ros2 launch audio_bringup doa_simulated.launch.py
  ```
- To simulate the drone doing wall detection, run: 
  ```
  ros2 launch audio_bringup wall_simulated.launch.py
  ```

### Experiments

- To do real-time sound source localization on the Crazyflie drone, run: 
  ```
  ros2 launch audio_bringup doa_real.launch.py
  ```
- To do real-time wall detection on the Crazyflie drone, run: 
  ```
  ros2 launch audio_bringup wall_real.launch.py
  ```
- To run a new measurement campaign, run: 
  ```
  python src/audio_bringup/audio_bringup/measurement_pipeline.py
  ```
- To convert bag file to csv: after editing the global variables in the node source code, 
  run: 
  ```
  ros2 run audio_bringup convert_bag_to_csv
  ```
  Note that this function is experimental and only used when the bag to csv conversion did not happen correctly during experiments. 

## Requirements (WIP)

List of requirements to run all nodes in this repo, continuously updated and will (hopefully) eventually be added to installation instructions.
- pyserial (for turntable handling)

Need to grant access to /dev/ttyACM0:

```sudo chmod 666 /dev/ttyACM0 ```

## Credits

We took inspiration from [this](https://roboticsbackend.com/package-organization-for-a-ros-stack-best-practices/) helpful article for the structure of this repo. 
