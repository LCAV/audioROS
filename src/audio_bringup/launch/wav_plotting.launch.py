#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
wav_plotting.launch.py: Read and plot wav file
"""

import sys

import launch

from audio_bringup.helpers import get_launch_description

LOG_LEVEL = "warn"

node_config = {
    "file": {"pkg": "audio_publisher"},
    "processor": {
        "pkg": "audio_stack", 
        "params": [{
            "filter_snr": 0,
            "min_freq": 100,
            "max_freq": 5000,
            "n_freqs": 1025 
        }]
    },
    "audio": {"pkg": "topic_plotter"},
}


def generate_launch_description():
    return get_launch_description(node_config, log_level=LOG_LEVEL)


def main(argv=sys.argv[1:]):
    ld = generate_launch_description()
    ls = launch.LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == "__main__":
    main()
