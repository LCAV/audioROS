#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
doa.launch.py: Launch the doa estimation and plotting pipeline.
"""

import sys

import launch

from audio_bringup.helpers import get_launch_description

LOG_LEVEL = "warn"

node_config = {
    "spectrum_estimator": {"params": [{"bf_method": "mvdr"}], "pkg": "audio_stack"},
    "doa_estimator": {"params": [{"combination_n": 3, "combination_method": "sum"}], "pkg": "audio_stack"},
    "audio": {"pkg": "topic_plotter"},
    "geometry": {"pkg": "topic_plotter"},
}

def generate_launch_description():
    return get_launch_description(node_config, log_level=LOG_LEVEL)


def main(argv=sys.argv[1:]):
    ld = generate_launch_description(node_config, log_level=LOG_LEVEL)
    ls = launch.LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == "__main__":
    main()
