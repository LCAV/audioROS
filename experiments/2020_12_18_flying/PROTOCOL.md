# Flying experiments

__Time__: Friday 18.12.2012, afternoon

__Recorded by__: Adrien Hoffet, Frederike Dümbgen

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
- Buzzer on drone, drone without foam on mics. 
- No stepper motors used; drone was flying.
- Measurement mic placed besides the setup, connected to USB soundcard. 
- Printed checkerboard used for optical flow. 
- Used LCAV Lenovo Thinkpad.

### Protocol
<!--
Checklist: 
- Sound level calibration
- Order of scripts run
- Times of battery exchange etc. 
- Start/end times of recordings, synchronization
-->

First round (no appendix or time appendix that will be manually removed): 

- Global duration set to 30, led to ca. 4s waiting time in end.
- First measurement: drone crashed in the very end, battery low.

Second round (appendix new): 

- Duration set to 26s.
- All measurements with same battery! start at 4.1, down to ca. 3.75.
- Last measurement: battery level quite low at start, rosbag error (topic not created yet, for the crazyflie/motors topic). still ran until the end. 

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

Soundcard settings unchanged from last time
Used 300Hz for audio deck rate. 
Very few communication problems compared to last times. 

### Data
<!--
Explain folder naming etc. 
-->

- as always

### Observations
<!--
Anything unusual that happened during the experiments, such as
- Background noise
- Connection problems, low data rates, etc. 
- Hardware (battery failures, broken parts, etc)
-->

Noticed that we used blocking "forward" command and changed this before restarting experiments. 

All experiments were filmed. videos can be found under vidoes/ folder on peronsl laptopt (not added to github because of space issues) 
