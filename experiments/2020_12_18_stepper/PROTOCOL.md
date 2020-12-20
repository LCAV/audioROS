# Flying experiments

__Time__: Friday 18.12.2012, evening

__Recorded by__: Frederike Dümbgen

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
- Buzzer on drone, drone with foam on mics. 
- Linear stepper motor used.
- Measurement mic placed above mic1, connected to USB soundcard. 
- Used LCAV Lenovo Thinkpad.

### Protocol
<!--
Checklist: 
- Sound level calibration
- Order of scripts run
- Times of battery exchange etc. 
- Start/end times of recordings, synchronization
-->

- 0,  51: stopped communicating, not saved
- 0, -51: stopped working around 20cm, restarted gateway
- 1,  51: some errors around 20, 30cm, otherwise ok
- 1, -51: started wrongly while still moving, needed to restart. 
          restart: stopped communicating around 20cm, restarted
- 2,  51: stopped working around 20cm
- 2, -51: stopped working at ca. 20cm, tried restart but didn't work. 
- 3,  51: stopped working around 20, restart
- 3, -51: stopped working in very beginning
- sweep_fast: 51: worked super well!
-            -51: stopped working at ca. 20cm
- None 51: stopped working at ca. 20cm

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

Used the params/flying.yaml parameters for gateway.
Used params.py file.
Soundcard settings unchanged from last time
Used 300Hz for audio deck rate. 

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

Many communication problems.
