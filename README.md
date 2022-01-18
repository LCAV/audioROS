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
rosdep update --include-eol-distros #needed for eloquent  
rosdep install --from-path src/ --rosdistro $ROS_DISTRO
colcon build --symlink-install
. install/local_setup.bash
```

If you want to setup the git precommit hooks for automatic formatting and jupyter outpoot removal, run
```
pre-commit install
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

### Analysis

This repo also contains functions to analyze the experimental data, grouped in the folder `python/`. Below table explains where to find what. 

Note that the notebooks use preprocessed data to speed up analysis. To process data for the first time, you can use the examples listed in the Makefile. For instance, to preprocess the stepper motor results of the crazyflie reported in the thesis, run
`make crayflie_stepper_thesis`. Similarly, for the e-puck results run `make epuck_results`. See `python/Makefile` for all options.

Descriptions:

| notebook         | description                                                               |
|------------------|---------------------------------------------------------------------------|
|  StepperAnalysis |  analysis of stepper motor experiments, angle and distance inference      |
|  DistanceFlying  |  analysis of experiments where drone approaches wall while playing sweeps |


| notebook            | datasets               | results                                                                          |
|---------------------|------------------------|----------------------------------------------------------------------------------|
|  StepperAnalysis    | 2021_07_08_stepper_fast|   calibration, distance and freq slice, matrices                                 |
|                     | 2021_07_27_epuck_wall  |   epuck results                                                                  |
|                     | 2021_10_07_stepper     |   new paper results?                                                             |
|  CleanupAnalysis    | 2021_07_08_stepper_fast|   all matrix results                                                             |
|  DistanceFlying     | 2021_10_12_flying      |   flying, distance heatmap results                                               |
|                     | 2021_11_23_demo        |   new paper results?                                                             |
|                     | 2021_05_04_flying      |   backup of experiments where each amplitude measurement has timestamp (binsel=3)|
|                     | 2021_04_30_stepper     |   used for calibration of above                                                  |
|  ApproachAngleFlying| 2021_05_04_linear      |   approach angle results thesis                                                  |
|                     | 2021_10_12_linear      |   new paper results?                                                             |
|  WallStudy          | --                     |   theoretical limits of wall detection algorithms, simulation results            |
|  DoaAnalysis        | 2021_10_12_doa_stepper |   new paper results?                                                             |
|                     | 2021_10_12_doa_flying  |   new paper results?                                                             |
|  DoaStudy           | --                     |   simulation results doa algorithms                                              |


## Requirements (WIP)

Need to grant access to /dev/ttyACM0:

```sudo chmod 666 /dev/ttyACM0 ```


## Credits

We took inspiration from [this](https://roboticsbackend.com/package-organization-for-a-ros-stack-best-practices/) helpful article for the structure of this repo.
