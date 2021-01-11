#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
convert_bag_to_csv.py:  

Convert a bag file to a csv file.
"""

import os
import signal
import subprocess

import rclpy

from audio_bringup.helpers import set_param, EXP_DIRNAME, CSV_DIRNAME

EXPERIMENT = "2020_12_18_stepper"

FILENAME = None # set to None to loop through all. 
#FILENAME = "nomotors_nosnr_props_mono3875_-51"
#FILENAME = "test_bag_file"

def main(args=None):
    rclpy.init(args=args)

    if FILENAME is not None:
        filenames = [FILENAME]
    else:
        filenames = []
        for (dirpath, dirnames, contents) in os.walk(os.path.join(EXP_DIRNAME, EXPERIMENT)):
            if 'metadata.yaml' in contents:
                filename = os.path.basename(dirpath)

                # make sure dirpath follows convention.
                assert dirpath == os.path.join(EXP_DIRNAME, EXPERIMENT, filename), dirpath

                filenames.append(filename)
                print('found bag folder:', filename)

    subprocess.run(['sudo', 'pkill', 'csv_writer'])
    subprocess.run(['sudo', 'pkill', 'ros2'])

    for filename in filenames: 
        print(f'treating {filename}...')
        bag_filename = os.path.join(EXP_DIRNAME, EXPERIMENT, filename) 
        csv_filename = os.path.join(EXP_DIRNAME, EXPERIMENT, CSV_DIRNAME, filename + ".csv")

        # TODO(FD) this would be cleaner with lifecycle nodes
        # (or with the feature ros2 node kill, which doesn't 
        # exist yet).
        # check that not multiple csv_writers are running

        csv_pid = subprocess.Popen(['ros2', 'run', 'topic_writer', 'csv_writer'])
        success = False
        while not success:
            print('trying to reset csv_writer...')
            success = set_param('/csv_writer', 'filename', '')

        subprocess.run(['ros2', 'bag', 'play', bag_filename])

        # once bag file playing is done, we continue with below. 
        set_param('/csv_writer', 'filename', csv_filename)


if __name__ == "__main__":
    main()
