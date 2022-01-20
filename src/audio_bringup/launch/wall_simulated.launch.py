#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
wall_simulated.launch.py: Launch the wall detection pipeline in simulation. 
"""

import sys

import launch

from audio_bringup.helpers import get_launch_description

LOG_LEVEL = "warn"

params_path = "config/wall_simulated.yaml"

node_config = {
    "crazyflie": {"pkg": "audio_simulation", "ros__parameters": [params_path],},
    "wall_approach_publisher": {"pkg": "audio_simulation"},
    "processor": {"pkg": "audio_stack", "ros__parameters": [params_path],},
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
