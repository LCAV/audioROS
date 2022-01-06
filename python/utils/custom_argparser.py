"""
custom_argparser.py: Command line argument parsers used by multiple scripts
"""

import argparse

def exp_parser(description=""):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument('--experiment_names', type=str, nargs='+',
            default = [], help='list of experiment names to process (folders inside "experiment_root".')
    parser.add_argument('--experiment_root', type=str,
        default='../datasets/', help='absolute or relative path to experiment folder.'
    )
    parser.add_argument('--platform', type=str, default='crazyflie', help='platform to use (crazyflie or epuck).'
    )
    return parser

def check_platform(args):
    from .constants import PLATFORM
    if PLATFORM != args.platform:
        print(PLATFORM)
        print(args.platform)
        raise ValueError("Make sure PLATFORM is changed in utils.constants before running this script.")
