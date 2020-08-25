#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
file.launch.py: Launch the processing pipeline from a prerecorded file. 
"""

import sys

import launch

from audio_bringup.helpers import get_launch_description 

LOG_LEVEL = 'warn'

node_config = {
    'file': {
        'params': [{'n_buffer': 2**8, 'publish_rate': 100}],
        'pkg': 'audio_publisher'
    },
    'doa_estimator': {
        'params': [{'bf_method':'mvdr'}],
        'pkg': 'audio_stack'
    },
    'correlator': {
        'params': [{'noise':'', 'frequency':'uniform', 'window':'tukey'}],
        'pkg': 'audio_stack'
    }
}

def generate_launch_description():
    return get_launch_description(node_config, log_level=LOG_LEVEL)

def main(argv=sys.argv[1:]):
    ld = generate_launch_description(node_config, log_level=LOG_LEVEL)
    ls = launch.LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return ls.run()

if __name__ == '__main__':
    main()
