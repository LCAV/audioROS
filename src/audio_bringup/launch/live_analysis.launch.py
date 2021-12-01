#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
wall_approach_dslice.launch.py:  Launch the processing pipeline from the Crazyflie CRTP stream.
"""

import sys

import launch

from audio_bringup.helpers import get_launch_description
import os
import yaml

LOG_LEVEL = "warn"
PARAMS_FILE = "params/analysis.yaml"
BAG_FILE_ROOT = ""
BAG_PLAYBACK = ""

node_config = {
    "wall_detection": {"pkg": "crazyflie_demo"},
    #"geometry": {"pkg": "topic_plotter"},
    "distribution": {"pkg": "topic_plotter"},
    #"status": {"pkg": "topic_plotter"},
}

# add parameters from file to node_config
with open(PARAMS_FILE) as f:
    config_dict = yaml.load(f, Loader=yaml.FullLoader)
    for key, values in config_dict.items():
        if key in node_config.keys():
            node_config[key].update(values)
        elif key == "logging":
            print("setting log level to", values)
            LOG_LEVEL = values
        elif key == "bag_playback":
            print("setting log level to", values)
            BAG_PLAYBACK = values

def generate_launch_description():
    if BAG_FILE_ROOT != "":
        for counter in range(100):
            bag_file = f"{BAG_FILE_ROOT}{counter}"
            if not os.path.exists(bag_file):
                break
        print(f"recording bag file at {bag_file}")
    else:
        bag_file = ""
    return get_launch_description(
        node_config, log_level=LOG_LEVEL, bag_filename=bag_file, bag_playback=BAG_PLAYBACK
    )


def main(argv=sys.argv[1:]):
    ld = generate_launch_description(node_config, log_level=LOG_LEVEL)
    ls = launch.LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == "__main__":
    main()
