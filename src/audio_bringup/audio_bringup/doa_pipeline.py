#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fixed_source.py: 

Experiment with a fixed loudspeaker playing some stationary signal.
"""

import os
import signal
import subprocess
import sys
import time

import numpy as np
from scipy.io import wavfile

sys.path.append(os.getcwd() + "/crazyflie-audio/python/")
from play_and_record import get_usb_soundcard_ubuntu
from signals import generate_signal

EXP_DIRNAME = os.getcwd() + "/experiments/"
EXTRA_DIRNAME = 'testing'
#EXTRA_DIRNAME = '2020_10_14_static'
TOPICS_TO_RECORD =  ['/audio/signals_f', '/geometry/pose_raw']
#TOPICS_TO_RECORD = ['--all'] 
CSV_DIRNAME = "csv_files/"
WAV_DIRNAME = "export/"

THRUST = 3000
DEGREE_LIST = [0, 20, 45]
SOURCE_LIST = ['random_linear', 'mono_linear']

# sound card settings
DURATION = 30 # seconds
FS_SOUNDCARD = 44100 # hz
N_MEAS_MICS = 1 # number of measurement mics


def get_filename(**params):
    source_flag = "None" if params.get("source") is None else params.get("source")
    props_flag = "" if params.get("props") else "no"
    snr_flag = "" if params.get("snr") else "no"
    motors_flag = "" if params.get("motors")>0 else "no"
    ending = "" if params.get("degree") == 0 else f"_{params.get('degree')}"
    fname = f"{motors_flag}motors_{snr_flag}snr_{props_flag}props_{source_flag}{ending}"
    return fname

def set_param(node_name, param_name, param_value):
    param_pid = subprocess.Popen(['ros2', 'param', 'set', node_name, param_name, param_value], stdout=subprocess.PIPE)
    print('waiting to set params')
    out_bytes, err = param_pid.communicate()
    out_string = out_bytes.decode("utf-8").strip()
    if out_string == "Set parameter successful":
        return True
    else:
        print("error:", out_string)
        return False

def get_active_nodes():
    param_pid = subprocess.Popen(['ros2', 'node', 'list'], stdout=subprocess.PIPE)
    out_bytes, err = param_pid.communicate()
    out_string = out_bytes.decode("utf-8").strip()
    return out_string


if __name__ == "__main__":

    active_nodes = get_active_nodes()
    assert '/csv_writer' in active_nodes
    assert '/gateway' in active_nodes

    # calibration:
    params_list = [
        {'motors': 0, 'snr': False, 'props': False, 'source':None, 'degree':0},
        {'motors': THRUST, 'snr': False, 'props': False, 'source':None, 'degree':0},
    ]
    # other experiments: 
    for degree in DEGREE_LIST:
        for source in SOURCE_LIST: 
            params_list += [
                {'motors': 0, 'snr': True, 'props': False, 'source':source, 'degree':degree},
                {'motors': 0, 'snr': False, 'props': False, 'source':source, 'degree':degree},
                {'motors': THRUST, 'snr': False, 'props': False, 'source':source, 'degree':degree},
                {'motors': THRUST, 'snr': True, 'props': False, 'source':source, 'degree':degree},
                {'motors': THRUST, 'snr': False, 'props': True, 'source':source, 'degree':degree},
                {'motors': THRUST, 'snr': True, 'props': True, 'source':source, 'degree':degree},
            ]

    sd = get_usb_soundcard_ubuntu(FS_SOUNDCARD, N_MEAS_MICS)

    extra_dirname = input(f'Enter experiment folder: (appended to {EXP_DIRNAME}, default:{EXTRA_DIRNAME})') or EXTRA_DIRNAME
    exp_dirname = os.path.join(EXP_DIRNAME, extra_dirname)
    csv_dirname = os.path.join(exp_dirname, CSV_DIRNAME)
    wav_dirname = os.path.join(exp_dirname, WAV_DIRNAME)

    for dirname in [exp_dirname, csv_dirname, wav_dirname]:
        if not os.path.exists(dirname):
            os.makedirs(dirname)
            print(f'created {dirname}')
        print(f'saving under {dirname}')

    # reset the csv writer
    timestamp = int(time.time())
    for params in params_list:
        answer = ''
        while not (answer in ['y', 'n']):
            answer = input(f'Start experiment with {params}? ([y]/n)') or 'y'
        if answer == 'n':
            sys.exit() 

        filename = get_filename(**params)
        bag_filename = os.path.join(exp_dirname, filename)
        csv_filename = os.path.join(csv_dirname, filename)
        out_signal = generate_signal(FS_SOUNDCARD, DURATION, signal_type=params['source'])

        while os.path.exists(bag_filename):
            answer = input(f'Path {filename} exists, append something? (default:{timestamp}, n to exit)') or timestamp

            if answer == 'n':
                sys.exit()

            filename = f'{filename}_{answer}'
            bag_filename = os.path.join(exp_dirname, filename)
            csv_filename = os.path.join(csv_dirname, filename)

        # TODO(FD) csv file and bag file will not be perfectly synchronized.
        # if that is necessary, we need to rewrite this section.

        # reset csv writer
        if not set_param('/csv_writer', 'filename', ''):
            sys.exit()

        # start recording bag file
        bag_pid = subprocess.Popen(['ros2', 'bag', 'record', '-o', bag_filename] + TOPICS_TO_RECORD)
        print('waiting to start bag record')
        time.sleep(1)

        # set thrust (or start hover)
        # ros2 param set /gateway all 43000 
        if not set_param('/gateway', 'all', str(params['motors'])):
            sys.exit()
        
        answer = ''
        while not (answer in ['y', 'n']):
            try:
                print('recording...')
                recording = sd.playrec(out_signal, blocking=True)
                print('...done')
                answer = 'y'
            except ValueError:
                answer = input('Make sure the audio is correctly connected! Press enter to try again. (or enter "n" to abort)')

        if answer == 'n':
            sys.exit()

        # when done playing sound: stop bag file
        bag_pid.send_signal(signal.SIGINT)

        # record the csv file
        if not set_param('/csv_writer', 'filename', csv_filename):
            sys.exit()

        # set thrust to 0 (or stop hover)
        if not set_param('/gateway', 'all', '0'):
            sys.exit()

        # save wav file.
        recording_float32 = recording.astype(np.float32)
        wav_filename = os.path.join(wav_dirname, filename) + '.wav'

        wavfile.write(wav_filename, FS_SOUNDCARD, recording_float32)
        print('wrote wav file as', wav_filename)
