#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
convert_all.py: 
"""

import argparse
import os
import subprocess
import signal
import time

from .csv_writer import TIMEOUT_S

def main(*args, **kwargs):

    parser = argparse.ArgumentParser(description='description')
    parser.add_argument('--dirname', dest='dirname', default='.') 
    args = parser.parse_args()

    for subdir, dirs, files in os.walk(args.dirname):
        if not 'metadata.yaml' in files:
            print('skipping', subdir)
            continue
        else:
            # remove the "./" from beginning of filename.
            filename = subdir[2:] 

        # TODO(FD) fix below.
        #process = subprocess.Popen(['ros2', 'run', 'topic_writer', 'csv_writer', '--ros-args', '-p', f'filename:={filename}'])
        subprocess.Popen(['ros2', 'run', 'topic_writer', 'csv_writer'])
        time.sleep(1)
        subprocess.run(['ros2', 'param', 'set', '/csv_writer', 'filename', filename])

        start_time = time.time()
        subprocess.run(['ros2', 'bag', 'play', filename])

        while (time.time() - start_time) < TIMEOUT_S:
            pass
        subprocess.run(['sudo', 'pkill', 'csv_writer'])
        print('convert_all: processing next file after timeout...')

if __name__ == '__main__':
    main()
