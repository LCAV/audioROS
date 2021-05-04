#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
wav_plotting.launch.py: Read and plot wav file
"""

import sys

import launch

from audio_bringup.helpers import get_launch_description

LOG_LEVEL = "warn"

MIN_FREQ = 100
MAX_FREQ = 10000
APPENDIX = "_7"

node_config = {
    "file": {"pkg": "audio_publisher", "params": [{"appendix": APPENDIX}]},
    "processor": {
        "pkg": "audio_stack",
        "params": [
            {
                "filter_snr": 0,
                "min_freq": MIN_FREQ,
                "max_freq": MAX_FREQ,
                "n_freqs": 1025,
            }
        ],
    },
    "audio": {
        "pkg": "topic_plotter",
        "params": [{"min_freq": MIN_FREQ, "max_freq": MAX_FREQ,}],
    },
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
