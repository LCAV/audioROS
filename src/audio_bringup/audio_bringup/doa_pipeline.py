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
import serial

sys.path.append(os.getcwd() + "/crazyflie-audio/python/")
from play_and_record import get_usb_soundcard_ubuntu
from signals import generate_signal
from serial_motors import SerialMotors

from crazyflie_crtp.commands import command_dict, buzzer_dict

EXP_DIRNAME = os.getcwd() + "/experiments/"
#EXTRA_DIRNAME = '2020_10_14_static_new'
#EXTRA_DIRNAME = '2020_10_30_dynamic_test'
#EXTRA_DIRNAME = '2020_10_30_dynamic'
#EXTRA_DIRNAME = '2020_10_30_dynamic_move'
#EXTRA_DIRNAME = '2020_11_03_sweep'
#EXTRA_DIRNAME = '2020_11_10_buzzer'

#EXTRA_DIRNAME = '2020_11_12_buzzer360'
#EXTRA_DIRNAME = '2020_11_12_speaker360'
#EXTRA_DIRNAME = '2020_11_13_speaker360'
#EXTRA_DIRNAME = '2020_11_18_speaker360'
#EXTRA_DIRNAME = '2020_11_19_wall'
#EXTRA_DIRNAME = '2020_11_20_wall'
#EXTRA_DIRNAME = '2020_11_23_wall'
#EXTRA_DIRNAME = '2020_11_23_wall2'
#EXTRA_DIRNAME = '2020_11_26_wall'
EXTRA_DIRNAME = '2020_11_27_wall_props'

TOPICS_TO_RECORD =  ['/audio/signals_f', '/geometry/pose_raw']
#TOPICS_TO_RECORD = ['--all'] 
CSV_DIRNAME = "csv_files/"
WAV_DIRNAME = "export/"


def get_filename(**params):
    source_flag = "None" if params.get("source") is None else params.get("source")
    props_flag = "" if params.get("props")==1 else "no"
    snr_flag = "" if params.get("snr")==1 else "no"
    motors = params.get("motors")
    motors_flag = "" if ((type(motors) == str) or (motors > 0)) else "no"
    ending = "" if params.get("degree") == 0 else f"_{params.get('degree')}"
    ending_distance = "" if params.get("distance", None) is None else f"_{params.get('distance')}"
    fname = f"{motors_flag}motors_{snr_flag}snr_{props_flag}props_{source_flag}{ending}{ending_distance}"
    return fname


def set_param(node_name, param_name, param_value):
    param_pid = subprocess.Popen(['ros2', 'param', 'set', node_name, param_name, param_value], stdout=subprocess.PIPE)
    print('waiting to set params:', param_name, param_value)
    out_bytes, err = param_pid.communicate()
    out_string = out_bytes.decode("utf-8").strip()
    if out_string == "Set parameter successful":
        return True
    else:
        print("set_param error:", out_string)
        return False


def execute_commands(command_list):
    for command in command_list:
        node, parameter, value, sleep = command
        print(f'execute {parameter}: {value} and sleep for {sleep}s...')
        set_param(node, parameter, str(value))
        time.sleep(sleep)


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
    if (source_type == 'soundcard') or (global_params['n_meas_mics'] > 0):
        sd = get_usb_soundcard_ubuntu(global_params['fs_soundcard'], global_params['n_meas_mics'])

        sound = np.zeros(10, dtype=float)
        if global_params['n_meas_mics'] > 0:
            if source_type == 'soundcard':
                print('playing and recording zero test sound...')
                sd.playrec(sound, blocking=True)
            else:
                print('recording zero test sound...')
                sd.rec(10, blocking=True)
        else:
            print('playing zero test sound...')
            sd.play(sound, blocking=True)

    for dirname in [exp_dirname, csv_dirname, wav_dirname]:
        if not os.path.exists(dirname):
            os.makedirs(dirname)
            print(f'created {dirname}')
        print(f'saving under {dirname}')

    previous_distance = 0
    timestamp = int(time.time())
    for param_i, params in enumerate(params_list):

        #### prepare filenames ####
        #answer = ''
        answer = 'y'
        while not (answer in ['y', 'n']):
            answer = input(f'start experiment with {params}? ([y]/n)') or 'y'
        if answer == 'n':
            continue
        print(f'starting experiment {param_i}/{len(params_list)} with {params}')

        if type(params['motors']) == str:
            if source_type == 'soundcard':
                raise ValueError('cannot play sound card and control motors at the same time')
            elif global_params['n_meas_mics'] > 0:
                raise ValueError('cannot record sound card and control motors at the same time')

        filename = get_filename(**params)
        bag_filename = os.path.join(exp_dirname, filename)
        csv_filename = os.path.join(csv_dirname, filename)

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

        #### prepare sound and turntable interfaces ####

        if source_type == 'soundcard':
            out_signal = generate_signal(global_params['fs_soundcard'], 
                    global_params['duration'], 
                    signal_type=params['source'], 
                    frequency_hz=global_params['freq_source'], 
                    min_dB=global_params['min_dB'], 
                    max_dB=global_params['max_dB'])
        elif source_type == 'buzzer':
            input(f'make sure buzzer plays {params["source"]} at frequency {global_params["freq_source"]}! Enter to continue')

        distance = params.get('distance', None)
        if (params['degree'] != 0) or (distance is not None):
            SerialIn = SerialMotors(verbose=False)

        #### prepare drone ####

        # set audio parameters
        set_param('/gateway', 'filter_snr_enable', str(params['snr']))
        set_param('/gateway', 'filter_prop_enable', str(params['props']))

        # TODO(FD) do this for all possible parameters 
        if params.get('min_freq', None) is not None:
            set_param('/gateway', 'min_freq', str(params['min_freq']))
        if params.get('max_freq', None) is not None:
            set_param('/gateway', 'max_freq', str(params['max_freq']))

        if distance is not None:
            delta = distance - previous_distance 
            if delta > 0:
                print('moving forward by', delta)
                SerialIn.move(delta)
            elif delta < 0:
                print('moving backward by', delta)
                SerialIn.move_back(delta)
            previous_distance = distance


        if params['degree'] != 360:
            print('turning by', params['degree'])
            SerialIn.turn(params['degree'])


        if (type(params['motors']) is int) and (params['motors'] > 0):
            success = set_param('/gateway', 'all', str(params['motors']))
            if not success:
                print('battery low!')
                if params['degree'] != 360:
                    print('turning back by', params['degree'])
                    SerialIn.turn_back(params['degree'])

        # TODO(FD) csv file and bag file will not be perfectly synchronized.
        # if that is necessary, we need to rewrite this section.
        # start recording csv file
        set_param('/csv_writer', 'filename', '')
        # start recording bag file
        bag_pid = subprocess.Popen(['ros2', 'bag', 'record', '-o', bag_filename] + TOPICS_TO_RECORD)
        print('started bag record')


        # play buzzer sound
        if source_type == 'buzzer-onboard':
            if params['source'] is not None:
                execute_commands(buzzer_dict[params['source']])

        if params['degree'] == 360:
            print('starting turn', params['degree'])
            SerialIn.turn(params['degree'])

        #### get measurements ####
        if source_type == 'soundcard':
            if global_params['n_meas_mics'] > 0:
                print('playing and recording sound...')
                recording = sd.playrec(out_signal, blocking=True)
            else:
                print('playing (not recording) sound...')
                sd.play(out_signal, blocking=True)

        # when we use the buzzer, we only record what the measurement mics get.    
        elif (source_type == 'buzzer-onboard') or (source_type == 'buzzer'):

            if global_params['n_meas_mics'] > 0:
                print(f'recording sound for {global_params["duration"]} seconds...')
                n_frames = global_params['duration'] * global_params['fs_soundcard']
                recording = sd.rec(n_frames, blocking=True)
            else:
                if type(params['motors']) == str:
                    print(f'executing motor commands:')
                    execute_commands(command_dict[params['motors']])
                else:
                    print(f'waiting for {global_params["duration"]} seconds...')
                    time.sleep(global_params['duration'])

            # end buzzer sound
            if source_type == 'buzzer-onboard':
                execute_commands(buzzer_dict['stop'])

        print('...done')

        # when done playing sound: stop bag file
        bag_pid.send_signal(signal.SIGINT)

        # record the csv file
        set_param('/csv_writer', 'filename', csv_filename)

        # set thrust to 0 (or stop hover)
        set_param('/gateway', 'all', '0')

        if (type(params['motors']) is int) and (params['motors'] > 0):
            print('wait for 1.5 minutes for batteries to recharge')
            time.sleep(90)

        # save wav file.
        if global_params['n_meas_mics'] > 0:
            recording_float32 = recording.astype(np.float32)
            wav_filename = os.path.join(wav_dirname, filename) + '.wav'
            wavfile.write(wav_filename, global_params['fs_soundcard'], recording_float32)
            print('wrote wav file as', wav_filename)

        # turn back
        SerialIn.turn_back(params['degree'])

    # after last experiment: move back to position 0
    if distance is not None:
        SerialIn.move_back(distance)
