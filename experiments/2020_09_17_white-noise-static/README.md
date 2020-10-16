<!-- create this document in each new experiments folder, calling it README.md -->
# EXPERIMENT NAME

__Time__: Thursday 17.09.2020, ca. 15.00-18.00

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

- Small speaker, center of bottom output mounted at 1.47 meters. 
- Drone with audio deck, height of microphones at 1.47 meters. 
- One measurement microphone at 1.73 meters, above the center of the drone, for control purposes.
- Source at 0 degrees corresponds to 180 degrees in the drone's frame. 
- Drone turned counter-clockwise by 20 and 45 degrees for different experiments.
- Distance speaker to center of drone: 1.9 meters. 


### Software
<!--
Checklist: 
- Sampling rate
- Motor thrust value 
- Audio files used
- Scripts used
- Other parameters used
-->

- Software versions used: version0.2 for crazyflie-audio, crazyflie-firmware, audioROS
- Sampling rate: 32000 kHz
- Motor thrust value: 43000, set using parameter motorPowerSet.m1-m4
- Used [crazyflie-audio/reaper/crazyflie_audio_measurements.RPP](https://github.com/LCAV/crazyflie-audio/blob/f7f3aa7760a42ed62dea02ee270952595b46cdaa/reaper/crazyflie_audio_measurements.RPP) file for speaker and measurement mic (commit: f7f3aa7760).

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
- Measurement microphone recorded during 45 degrees experiments. 
- Order of scripts:
  - `ros2 run crazyflie_crtp gateway`: start getting audio data
  - `ros2 param set ...`: set `filter_snr_enable` etc., set motor speeds
  - `*_bkp`: had to run this experiment again because forgot to record on soundcard side.
  - `*_crash`: had to run this experiment again because drone stopped sending data half-way.
  - run at the same time: `ros2 bag record -o ...`  and play (+record) the sound through the soundcard 
  - stop bag file recording as soon as the sound reaches its end.

### Data
<!-- Explain folder naming etc.  -->

- `export/`: Measurement microphone recordings.
- `<motors flag>_<snr filter flag>_<propeller filter flag>_<source flag>`: Audio deck recordings, e.g. `motors_nosnr_noprops_nosource`: propellers on, `filter_snr_enable` set to 0, `filter_props_enable` set to 0, no source playing.
- `photos`: Photos of the experiment setup.

### Observations
<!--
Anything unusual that happened during the experiments, such as
- Background noise
- Connection problems, low data rates, etc. 
- Hardware (battery failures, broken parts, etc)
-->

- Needed to change batteries 3 times. 
- Needed to restart the drone a few times because it stopped sending any valid audio data. 
