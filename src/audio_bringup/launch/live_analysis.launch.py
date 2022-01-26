#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
live_analysis.launch.py: replay and analyse bag files from past demo experiments.
"""

import sys

from launch import LaunchService
from launch.actions import DeclareLaunchArgument

from audio_bringup.helpers import get_launch_description, parse_params_file

PARAMS_FILE = "params/live_analysis.yaml"


def generate_launch_description():
    node_config = {
        "wall_mapper": {"pkg": "audio_gtsam"},
        "wall_detection": {"pkg": "crazyflie_demo"},  # make sure DRONE=1!
        "pose_synch": {"pkg": "audio_stack"},
        # "time": {"pkg": "topic_plotter"},
        "geometry": {"pkg": "topic_plotter"},
        "distribution": {"pkg": "topic_plotter"},
    }
    for node in node_config.keys():
        node_config[node]["ros__parameters"] = [PARAMS_FILE]
    launch_options_dict = parse_params_file(PARAMS_FILE)
    return get_launch_description(node_config, **launch_options_dict)


def main(argv=sys.argv[1:]):
    ld = generate_launch_description()
    ls = launch.LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == "__main__":
    main()
