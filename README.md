# audioROS

![The Crazylie drone and e-puck robot in front of the EPFL logo](doc/epuck-drone.png)

This repository contains the ROS2-powered stack to do audio processing for sound-based localization on robots. The system was tested on the Crazyflie drone 2.1 and the e-puck2 robot, shown in the image above.

## Installation

The repository was built for Ubuntu 20.04 (Focal Fossa) with ROS2 (Galactic) and Python 3.8.12. 

To install, make sure to clone this repo including submodules (e.g. datasets) by running
```
git clone --recurse-submodules https://github.com/LCAV/audioROS
```

To install all packages contained in this repo, including dependencies, run (from the root of this repository):
```
sudo apt install python3-rosdep python3-colcon-common-extensions
sudo rosdep init
cp 19-custom.list /etc/ros/rosdep/sources.list.d/ #might need sudo here
rosdep update 
rosdep install --from-path src/ --rosdistro $ROS_DISTRO
colcon build --symlink-install
. install/local_setup.bash
```

Update from May 25 2022: because of an inconsistency in matplotlib, in order to run all of the 
analysis notebooks (using latex rendering), the following two
non-standard libraries had to be installed:
```
sudo apt install cm-super dvipng
```

## Contents

The stack is separated into the following modules:

- `audio_interfaces` (C++): Custom ROS message definintions.
- `audio_bringup` (python): Pipeline for recording measurements (`measurement_pipeline.py`) and launch files.
- `audio_gtsam` (python): Classes to build a factor graph from audio and pose measurements.
- `audio_stack` (python): Read and process audio signals.
- `audio_simulation` (python): Simulated audio using pyroomacoustics.
- `audio_publisher` (python): Publish signals from file or computer's audio input stream.
- `crazyflie_crtp` (python): Publish audio signals received over CRTP from Crazyflie drone.
- `crazyflie_demo` (python): Demo of drone detecting and avoiding walls.
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
  ros2 run audio_bringup measurement_pipeline
  ```

### Analysis

This repo also contains functions to analyze the experimental data, grouped in the folder `python/`. Below table explains where to find what.

Note that the notebooks use preprocessed data to speed up analysis. To process data for the first time, you can use the examples listed in the Makefile. For instance, to preprocess the stepper motor results of the crazyflie reported in the thesis, run
`make crayflie_stepper_thesis`. Similarly, for the e-puck results run `make epuck_results`. See `python/Makefile` for all options.

Descriptions:

| notebook            | datasets                 | results                                                  |
|---------------------|--------------------------|----------------------------------------------------------|
|  StepperAnalysis    | `2021_07_08_stepper_fast`|   echolocation, distance and frequency slice, matrices   |
|                     | `2021_07_27_epuck_wall  `|   echolocation, epuck results                            |                                                                                                                                                                                                 
|  CleanupAnalysis    | `2021_07_08_stepper_fast`|   echolocation, matrix cleaning results                  |
|  DistanceFlying     | `2021_10_12_flying      `|   echolocation, drone flying towards walls               |
|                     | `2021_11_23_demo        `|   echolocation, drone avoiding walls                     |
|                     | `2022_01_27_demo        `|   echolocation, drone avoiding whiteboards               |
|  ApproachAngleFlying| `2021_05_04_linear      `|   echolocation, approach angle results                   |
|  WallStudy          | `--                     `|   echolocation, simulation results                       |
|  DoaAnalysis        | `2021_10_12_doa_stepper `|   doa, experiments on stepper motor                      |
|                     | `2021_10_12_doa_flying  `|   doa, experiments with hovering drone                   |
|  DoaStudy           | `--                     `|   doa, simulation results                                |


## Credits

We took inspiration from [this](https://roboticsbackend.com/package-organization-for-a-ros-stack-best-practices/) helpful article for the structure of this repo.

## References

Please refer to the below publications for more information.

```
F. Dümbgen, A. Hoffet, A. Scholefield, and M. Vetterli, "Blind as a bat: audible 
echolocation on tiny robots", submitted to IEEE/RSJ International Conference on 
Intelligent Robotis and Systems, 2022.
```

```
F. Dümbgen, "Blind as a bat: spatial perception without sight", Ph.D. disseration, 
École Polytechnique Fédérale de Lausanne, 2021.
```
