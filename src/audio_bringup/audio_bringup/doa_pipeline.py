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

sys.path.append(os.getcwd() + "/crazyflie-audio/python/")
from play_and_record import get_usb_soundcard_ubuntu
from signals import generate_signal

BAG_DIRNAME = os.getcwd() + "/experiments/"
#TOPICS_TO_RECORD =  ['/audio/signals_f', '/geometry/poseraw']
TOPICS_TO_RECORD = ['--all'] 
CSV_DIRNAME = os.getcwd() + "/csv_files/"

def get_filename(**params):
    source_flag = params.get("source") 
    props_flag = "" if params.get("props") else "no"
    snr_flag = "" if params.get("snr") else "no"
    motors_flag = "" if params.get("motors")>0 else "no"
    ending = "" if params.get("degree") == 0 else f"_{params.get('degree')}"
    fname = f"{motors_flag}motors_{snr_flag}snr_{props_flag}props_{source_flag}{ending}"
    return fname

def set_param(node_name, param_name, param_value):
    param_pid = subprocess.Popen(['ros2', 'param', 'set', node_name, param_name, param_value], stdout=subprocess.PIPE)
    print('waiting to set params')
    out, err = param_pid.communicate()
    print('done:', out, err)
    return

if __name__ == "__main__":
    params_list = [
            {'motors': 3000, 'snr': False, 'props': False, 'source':'random', 'degree':0},
    ]

    fs = 44100 # hz
    duration = 5 # seconds
    n_channels = 1 # number of measurement mics
    sd = get_usb_soundcard_ubuntu(fs, n_channels)

    extra_dirname = input(f'Enter experiment folder: (appended to {BAG_DIRNAME})') 
    BAG_DIRNAME = os.path.join(BAG_DIRNAME, extra_dirname)
    print('saving under', BAG_DIRNAME)
    if not os.path.exists(BAG_DIRNAME):
        os.makedirs(BAG_DIRNAME)
        print(f'created {BAG_DIRNAME}')
    if not os.path.exists(CSV_DIRNAME):
        os.makedirs(CSV_DIRNAME)
        print(f'created {CSV_DIRNAME}')


    csv_pid = subprocess.Popen(['ros2', 'run', 'topic_writer', 'csv_writer'])
    print('waiting to start csv wrtier')
    time.sleep(1)

    set_param('/csv_writer', 'dirname', CSV_DIRNAME)

    for params in params_list:
        answer = ''
        while not (answer in ['y', 'n']):
            answer = input(f'Start experiment with {params}? ([y]/n)') or 'y'

        if answer == 'n':
            sys.exit() 

        filename = get_filename(**params)
        bag_filename = os.path.join(BAG_DIRNAME, filename)
        out_signal = generate_signal(fs, duration, signal_type=params['source'])

        while os.path.exists(bag_filename):
            timestamp = int(time.time())
            answer = input(f'Path {filename} exists, append something? (default:{timestamp}, n to exit)') or timestamp

            if answer == 'n':
                sys.exit()

            filename = f'{filename}_{answer}'
            bag_filename = os.path.join(BAG_DIRNAME, filename)

        # set thrust (or start hover)
        # ros2 param set /gateway all 43000 
        set_param('/gateway', 'all', str(params['motors']))

        ########### OPTION 1 ###########
        # start playing sound + recording
        #recording = sd.playrec(out_signal, blocking=False)
        #start_time = time.time()

        # start recording bag file
        #bag_pid = subprocess.Popen(['ros2', 'bag', 'record', '-o', filename] + TOPICS_TO_RECORD)
        #while (time.time() - start_time) < duration:
        #    pass
        ################################

        ########### OPTION 2 ###########
        # start recording bag file
        bag_pid = subprocess.Popen(['ros2', 'bag', 'record', '-o', bag_filename] + TOPICS_TO_RECORD)
        print('waiting to start bag record')
        time.sleep(1)
        
        recording = sd.playrec(out_signal, blocking=True)
        #################################

        # when done playing sound: stop bag file
        bag_pid.send_signal(signal.SIGINT)
        print('waiting to stop bag file')
        time.sleep(1)

        set_param('/csv_writer', 'filename', filename)

        # set thrust to 0 (or stop hover)
        set_param('/gateway', 'all', '0')

    # TODO(FD) for some reason we do not manage to stop the csv writer
    # by running the below command. The correct way of doing this
    # would probably be through a life cycle node
    csv_pid.send_signal(signal.SIGINT)
    print('waiting to stop csv file')
    time.sleep(1)
