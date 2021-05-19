#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
doa_real.launch.py:  Launch the processing pipeline from the Crazyflie CRTP stream.
"""

import sys

import launch

from audio_bringup.helpers import get_launch_description

LOG_LEVEL = "warn"
BAG_FILE = ""

node_config = {
    "gateway": {"params": [{"filter_snr_enable": 1}], "pkg": "crazyflie_crtp"},
    "spectrum_estimator": {"params": [{"bf_method": "das"}], "pkg": "audio_stack"},
    "doa_estimator": {"pkg": "audio_stack"},
    "csv_writer": {"pkg": "topic_writer"},
    "audio": {"pkg": "topic_plotter"},
    "doa": {"pkg": "topic_plotter"},
    "geometry": {"pkg": "topic_plotter"},
}


def generate_launch_description():
    return get_launch_description(
        node_config, log_level=LOG_LEVEL, bag_filename=BAG_FILE
    )


def main(argv=sys.argv[1:]):
    ld = generate_launch_description(node_config, log_level=LOG_LEVEL)
    ls = launch.LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == "__main__":
    main()
