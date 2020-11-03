#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
crazyflie.launch.py:  Launch the processing pipeline from the Crazyflie CRTP stream.
"""

import sys

import launch

from audio_bringup.helpers import get_launch_description

LOG_LEVEL = "info"
BAG_FILE = "test_bagfile"

PARAMS_GATEWAY = [
    {
        "all": 0,
        "filter_snr_enable": 0,
    }
]
PARAMS_SPECTRUM = [
    {
        "bf_method": "mvdr"
    }
]

node_config = {
    "gateway": {"params": PARAMS_GATEWAY, "pkg": "crazyflie_crtp"},
    "spectrum_estimator": {"params": PARAMS_SPECTRUM, "pkg": "audio_stack"},
    "doa_estimator": {"pkg": "audio_stack"},
    "csv_writer": {"pkg": "topic_writer"},
    "audio": {"pkg": "topic_plotter"},
    "doa": {"pkg": "topic_plotter"},
    "geometry": {"pkg": "topic_plotter"},
}


def generate_launch_description():
    return get_launch_description(node_config, log_level=LOG_LEVEL, bag_filename=BAG_FILE)


def main(argv=sys.argv[1:]):
    ld = generate_launch_description(node_config, log_level=LOG_LEVEL)
    ls = launch.LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == "__main__":
    main()
