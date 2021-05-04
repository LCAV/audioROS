#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
buzzer_test.launch.py: 
"""

import os
import sys

import launch

from audio_bringup.helpers import get_launch_description

params_file = os.path.join("params", "buzzer_test.yaml")

node_config = {
    "gateway": {"params": [params_file], "pkg": "crazyflie_crtp"},
    "audio": {"pkg": "topic_plotter"},
}


def generate_launch_description():
    return get_launch_description(node_config)


def main(argv=sys.argv[1:]):
    ld = generate_launch_description(node_config)
    ls = launch.LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == "__main__":
    main()
