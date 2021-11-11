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
BAG_FILE_ROOT = "experiments/2021_11_11_test/linear"
PARAMS_FILE = "params/flying.yaml"

node_config = {
    "gateway": {"pkg": "crazyflie_crtp"},
    "wall_detection": {"pkg": "crazyflie_demo"},
    "geometry": {"pkg": "topic_plotter"},
    "distribution": {"pkg": "topic_plotter"},
    "status": {"pkg": "topic_plotter"},
}

# add parameters from file to node_config
with open(PARAMS_FILE) as f:
    config_dict = yaml.load(f, Loader=yaml.FullLoader)
    for key, values in config_dict.items():
        node_config[key].update(values)


def generate_launch_description():
    for counter in range(100):
        bag_file = f"{BAG_FILE_ROOT}{counter}"
        if not os.path.exists(bag_file):
            break
    print(f"recording bag file at {bag_file}")

    return get_launch_description(
        node_config, log_level=LOG_LEVEL, bag_filename=bag_file
    )


def main(argv=sys.argv[1:]):
    ld = generate_launch_description(node_config, log_level=LOG_LEVEL)
    ls = launch.LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == "__main__":
    main()
