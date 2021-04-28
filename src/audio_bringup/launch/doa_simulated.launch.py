#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
doa_simulated.launch.py: Launch the doa pipeline in simulation. 
"""

import sys

import launch

from audio_bringup.helpers import get_launch_description

LOG_LEVEL = "warn"

node_config = {
    "crazyflie": {
        "pkg": "audio_simulation",
        "params": [{"buzzer_freq": 0, "source_type": "random"}],
    },
    "linear_pose_publisher": {"pkg": "audio_simulation"},
    "processor": {
        "pkg": "audio_stack",
        "params": [{"min_freq": 1000, "max_freq": 5000}],
    },
    "spectrum_estimator": {"pkg": "audio_stack"},
    "doa_estimator": {"pkg": "audio_stack"},
    # "audio": {"pkg": "topic_plotter"},
    # "doa": {"pkg": "topic_plotter"}, is very slow
    "geometry": {"pkg": "topic_plotter"},
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
