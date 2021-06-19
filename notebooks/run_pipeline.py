#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
argparse.py: 
"""

import argparse, os

DESCRIPTION = """
    WIP: 
    Run full processing pipeline for new stepper motor experiments.
"""


def dir_path(string):
    if os.path.isdir(string):
        return string
    else:
        raise NotADirectoryError(string)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=DESCRIPTION, epilog="")
    parser.add_argument("--path", type=dir_path, help="name of experiments directory")
    args = parser.parse_args()
    base = os.path.basename(args.path)
    print("base:", base)
    print(args.path.split(base)[1])
