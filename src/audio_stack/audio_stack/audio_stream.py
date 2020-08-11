#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
audio_stream.py: 
"""
import sys

import sounddevice as sd

def select_audio_device():
    print(sd.query_devices())
    selected = input('select audio device from list above (c to cancel)')
    if selected == 'c':
        sys.exit()
    elif selected == '':
        selected = 'default'
    sd.default.device = selected #'digital_output'

def callback(indata, frames, time, status): 
    if status: 
        print(status)
    # indata is of shape n_buffer x n_mics
    print(indata.shape)

if __name__ == "__main__":
    duration = 1 # seconds

    sd.default.samplerate = 32000 #44100
    n_buffer = 2**10
    n_mics = 4

    max_trials = 10
    i = 0
    while i < max_trials:

        select_audio_device()

        try:
            with sd.InputStream(channels=n_mics, callback=callback, blocksize=n_buffer):
                sd.sleep(int(duration*1000)) # input is in miliseconds.
        except Exception as e:
            print('Error:', e)

        i += 1

    print('reached maximum of trials.')
    sys.exit()
