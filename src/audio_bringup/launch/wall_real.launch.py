#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
wall_simulated.launch.py: Launch the wall detection pipeline in simulation. 
"""

import sys

import launch

from audio_bringup.helpers import get_launch_description

LOG_LEVEL = "warn"

node_config = {
    "gateway": {
        "pkg": "crazyflie_crtp", 
        "params": [{
            "buzzer_effect": 12, 
            "buzzer_freq": 2000, 
            "filter_snr_enable": 2,
            "min_freq": 1000,
            "max_freq": 5000
        }]
    },
    "audio": {"pkg": "topic_plotter"},
    "geometry": {"pkg": "topic_plotter"},
    "wall": {"pkg": "topic_plotter"},
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
