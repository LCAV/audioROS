<!-- create this document in each new experiments folder, calling it README.md -->
# EXPERIMENT NAME

__Time__: Tuesday 10.11.2020, ca. 4pm - 5pm

__Recorded by__: Adrien Hoffet, Frederike Dümbgen 

__Place__: BC324

## Description

###  Hardware

Drone with buzzer deck playing at 4100 Hz, situated at ca. 50 cm from drone recording

<!--
Checklist: 
- Speaker type
- Microphone type
- Reference angle for DOA
- Distance speaker-mic etc. 
-->

### Software
<!--
Checklist: 
- Sampling rate
- Motor thrust value 
- Audio files used
- Scripts used
- Other parameters used
-->

### Protocol

- Adrien set the buzzer frequency from his laptop for the whole duration of experiments
- Rest of pipeline run from Frederike's laptop. (python src/audio_bringup/audio_bringup/doa_pipeline.py)
<!--
Checklist: 
- Sound level calibration
- Order of scripts run
- Start/end times of recordings, synchronization
-->

### Data
<!-- Explain folder naming etc.  -->

- 0: source at 0 degrees
- 90: source at 90 degrees
- 45: source at 0-90 degrees and back

Timing: 
- 45, no motors: to 90 degrees in ca. 15 seconds, back in ca. 15 seconds, then stand still.
- 45, motors: to 90 degrees in ca. 10 seconds, hold for 5 seconds, back in ca. 10 seconds, then stand still.

### Observations

- Needed to redo experiment at 0 degrees with motors on because drone flew away

<!--
Anything unusual that happened during the experiments, such as
- Background noise
- Connection problems, low data rates, etc. 
- Hardware (battery failures, broken parts, etc)
-->
