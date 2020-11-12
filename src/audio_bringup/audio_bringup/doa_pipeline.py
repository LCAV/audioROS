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
#EXTRA_DIRNAME = '2020_10_14_static_new'
#EXTRA_DIRNAME = '2020_10_30_dynamic_test'
#EXTRA_DIRNAME = '2020_10_30_dynamic'
#EXTRA_DIRNAME = '2020_10_30_dynamic_move'
#EXTRA_DIRNAME = '2020_11_03_sweep'
#EXTRA_DIRNAME = '2020_11_10_buzzer'

#EXTRA_DIRNAME = '2020_11_12_buzzer360'
EXTRA_DIRNAME = '2020_11_12_speaker360'
TOPICS_TO_RECORD =  ['/audio/signals_f', '/geometry/pose_raw']
#TOPICS_TO_RECORD = ['--all'] 
CSV_DIRNAME = "csv_files/"
WAV_DIRNAME = "export/"

def get_filename(**params):
    source_flag = "None" if params.get("source") is None else params.get("source")
    props_flag = "" if params.get("props")==1 else "no"
    snr_flag = "" if params.get("snr")==1 else "no"
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
    extra_dirname = input(f'enter experiment folder: (appended to {EXP_DIRNAME}, default:{EXTRA_DIRNAME})') or EXTRA_DIRNAME
    exp_dirname = os.path.join(EXP_DIRNAME, extra_dirname)
    csv_dirname = os.path.join(exp_dirname, CSV_DIRNAME)
    wav_dirname = os.path.join(exp_dirname, WAV_DIRNAME)

    sys.path.append(exp_dirname)
    from params import global_params, params_list
    print(f'loaded parameters from {exp_dirname}/params.py')

    active_nodes = get_active_nodes()
    assert '/csv_writer' in active_nodes
    assert '/gateway' in active_nodes


    source_type = global_params.get('source_type', 'soundcard')
    if (source_type == 'soundcard'):
        sd = get_usb_soundcard_ubuntu(global_params['fs_soundcard'], global_params['n_meas_mics'])

        sound = np.zeros(10, dtype=float)
        print('playing zero test sound...')
        sd.play(sound, blocking=True)

    for dirname in [exp_dirname, csv_dirname, wav_dirname]:
        if not os.path.exists(dirname):
            os.makedirs(dirname)
            print(f'created {dirname}')
        print(f'saving under {dirname}')

    # reset the csv writer
    timestamp = int(time.time())
    for params in params_list:
        answer = ''
        #answer = 'y'
        while not (answer in ['y', 'n']):
            answer = input(f'start experiment with {params}? ([y]/n)') or 'y'
            if params['source'] == 'buzzer':
                answer = input(f'Make sure buzzer is on!') or 'y'
        if answer == 'n':
            continue

        filename = get_filename(**params)
        bag_filename = os.path.join(exp_dirname, filename)
        csv_filename = os.path.join(csv_dirname, filename)

        if source_type == 'soundcard':
            out_signal = generate_signal(global_params['fs_soundcard'], 
                    global_params['duration'], 
                    signal_type=params['source'], 
                    frequency_hz=global_params['freq_source'], 
                    min_dB=global_params['min_dB'], 
                    max_dB=global_params['max_dB'])

        answer = 'y'
        while os.path.exists(bag_filename):
            answer = input(f'Path {filename} exists, append something? (default:{timestamp}, n to skip)') or timestamp

            if answer == 'n':
                break

            filename = f'{filename}_{answer}'
            bag_filename = os.path.join(exp_dirname, filename)
            csv_filename = os.path.join(csv_dirname, filename)

        if answer == 'n':
            continue

        # TODO(FD) csv file and bag file will not be perfectly synchronized.
        # if that is necessary, we need to rewrite this section.

        # reset csv writer
        set_param('/csv_writer', 'filename', '')

        # set audio parameters
        set_param('/gateway', 'filter_snr_enable', str(params['snr']))
        set_param('/gateway', 'filter_prop_enable', str(params['props']))

        # set thrust (or start hover)
        set_param('/gateway', 'all', str(params['motors']))

        # start recording bag file
        bag_pid = subprocess.Popen(['ros2', 'bag', 'record', '-o', bag_filename] + TOPICS_TO_RECORD)
        print('waiting to start bag record')
        time.sleep(1)
        
        print('source type:', source_type)
        if source_type == 'soundcard':
            if global_params['n_meas_mics'] > 0:
                print('playing and recording sound...')
                recording = sd.playrec(out_signal, blocking=True)
            else:
                print('playing (not recording) sound...')
                sd.play(out_signal, blocking=True)
        # when we use the buzzer, we only record what the measurement mics get.    
        elif source_type == 'buzzer':
            if global_params['n_meas_mics'] > 0:

                # TODO(FD) test this earlier.
                if type(params['motors']) == str:
                    raise ValueError('motors and source_type parameters incompatible')

                print(f'recording sound for {global_params["duration"]} seconds...')
                n_frames = global_params['duration'] * global_params['fs_soundcard']
                recording = sd.rec(n_frames, blocking=True)
            else:
                print(f'waiting for {global_params["duration"]} seconds...')
                if type(params['motors']) == str:
                    # TODO(FD): execute the sequence of motor commands.
                    raise NotImplementedError(params['motors'])
                    pass
                else:
                    time.sleep(global_params['duration'])
        print('...done')

        # when done playing sound: stop bag file
        bag_pid.send_signal(signal.SIGINT)

        # record the csv file
        set_param('/csv_writer', 'filename', csv_filename)

        # set thrust to 0 (or stop hover)
        set_param('/gateway', 'all', '0')

        # save wav file.
        if (global_params['n_meas_mics'] > 0):
            recording_float32 = recording.astype(np.float32)
            wav_filename = os.path.join(wav_dirname, filename) + '.wav'
            wavfile.write(wav_filename, global_params['fs_soundcard'], recording_float32)
            print('wrote wav file as', wav_filename)
