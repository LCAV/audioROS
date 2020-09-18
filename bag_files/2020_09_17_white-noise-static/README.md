<!-- create this document in each new experiments folder, calling it README.md -->
# EXPERIMENT NAME

__Time__: Thursday 17.09.2020

__Recorded by__: Frederike Dümbgen (ROS), Adrien Hoffet (Soundcard)

__Place__: Audio room INR018, EPFL

## Description

###  Hardware
<!--
Checklist: 
- Speaker type
- Microphone type
- Reference angle for DOA
- Distance speaker-mic etc. 
-->

- Small speaker, center of bottom output mounted at TODO:HEIGHT meters. 
- Drone with audio deck, height of microphones at TODO:HEIGHT meters. 
- One measurement microphone at TODO:HEIGHT meters, above the center of the drone, for control purposes.
- Source at 0 degrees corresponds to 180 degrees in the drone's frame. 
- Drone turned counter-clockwise by 20 and 45 degrees for different experiments.
- Distance speaker to center of drone: 190 cm. 


### Software
<!--
Checklist: 
- Sampling rate
- Motor thrust value 
- Audio files used
- Scripts used
- Other parameters used
-->

- Software versions used: TODO:TAG for crazyflie-audio, crazyflie-firmware, audioROS
- Sampling rate: 32000 kHz
- Motor thrust value: 43000, set using parameter motorPowerSet.m1-m4
- 

### Protocol
<!--
Checklist: 
- Sound level calibration
- Order of scripts run
- Start/end times of recordings, synchronization
-->

- Sound level calibration: measured at the measurement microphone location, during the experiments at 45 degrees.
  - Maximum source: 80 dB
  - Propeller noise: 73.5 dB
- Measurement microphone recorded during motors_nosnr_noprops_nosource, nomotors_nosnr_noprops_source, motors_nosnr_noprops_source. 
- Order of scripts:
  - `ros2 run crazyflie_crtp gateway`: start getting audio data
  - `ros2 param set ...`: set filter_snr_enable etc., set motor speeds
  - run at the same time: `ros2 bag record -o ...`  and play (+record) the sound through the soundcard 
  - stop bag file recording as soon as the sound reaches its end.

### Data
<!--
Explain folder naming etc. 
-->

- `<motors flag>_<snr filter flag>_<propeller filter flag>_<source flag>`, e.g. `motors_nosnr_noprops_nosource`: propellers on, `filter_snr_enable` set to 0, `filter_props_enable` set to 0, no source playing.
- `photos`: Photos of the experiment setup

### Observations
<!--
Anything unusual that happened during the experiments, such as
- Background noise
- Connection problems, low data rates, etc. 
- Hardware (battery failures, broken parts, etc)
-->

- Needed to change batteries 3 times. 
- Needed to restart the drone a few times because it stopped sending any valid audio data. 
