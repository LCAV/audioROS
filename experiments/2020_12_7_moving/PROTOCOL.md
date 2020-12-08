<!-- create this document in each new experiments folder, calling it README.md -->
# Moving wall detection 

__Time__: Monday 07.12.2020, afternoon

__Recorded by__: Frederike Duembgen

__Place__: BC325

## Description

###  Hardware
<!--
Checklist: 
- Speaker type
- Microphone type
- Motors for linear/rotational movement
- Computer
- Drone type, decks used
- Soundcard
-->

- Crazyflie 2.1, buzzer on the drone, audiodeck, battery-powered 
- 4 microphones on drone, one measurement microphone (1cm above mic)
- linear (short) and rotational motors
- run on LCAV thinkpad computer (Ubuntu18, ROS Eloquent)
- usb soundcard with external power

### Protocol
<!--
Checklist: 
- Sound level calibration
- Order of scripts run
- Times of battery exchange etc. 
- Start/end times of recordings, synchronization
-->
- no sound level calibration
- used `audio_bringup/doa_pipeline.py`
order (1: source at 3kHz, 2: source at 4 kHz, 3: source at 8kHz, 4: no source. X (forward) X' (backward))
- First round: 1, 1', 2, 2', 3, 3', 4, 4' 
- battery exchange
- Second round: 1, 1', 2, 2'
- battery exchange
- Third round: 4, 3'

### Parameters
<!--
Checklist: 
If available:
- parameters file location
- soundcard settings

Otherwise: 
- Sampling rate
- Motor thrust value 
- Audio files used
- Scripts used
- Other parameters used
-->
- soundcard: compliant mode, Hi-Z
- parameter list can be read from params.py


### Data
<!--
Explain folder naming etc. 
-->
- naming as usual
- renamed manually `motors_..._mono4156_-51` to `nomotors_...` because it was a typo.
- not using `motors_..._None_-51` because it is a duplicate and does not match the general pattern of motors forward, no motors backwards. 

### Observations
<!--
Anything unusual that happened during the experiments, such as
- Background noise
- Connection problems, low data rates, etc. 
- Hardware (battery failures, broken parts, etc)
-->

First round: 
- 1  stopped working at ca. 20cm, continued running and turned off propellers after a while
- 1' worked ok 
- 2  old frequency in beginning, stopped at ca. 20cm, turned off propellers. 
- 2' stopped working at timestamp 109592
- 3  super good run
- 3' failed at ca. 20cm, timestamp 113402
- 4  stopped working around 20cm, timestamp 95427
- 4' stopped working at 158541, also run with wrong command at first and moved back manually. Could lead to (milimeter-level) inaccuracies in distances from here on. 
Second round:
- 1  OK
- 1' stopped at 342744
- 2  OK, only a few warnings mid-way. Battery low towards end -> higher propeller speed.
- 2' stopped around 30cm, 118070. Saved this manually, and potentially added some faulty extra rows! Also lost the wav file of this one because the script crashed.  
Third round: 
- 4  OK, only a few warnings in ca. 1/3
- 3' super good run
