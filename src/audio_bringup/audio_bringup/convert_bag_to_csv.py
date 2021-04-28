#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
convert_bag_to_csv.py:  

Convert bag files to csv files. This script needs to be run carefully as the csv_writer nodes are sometimes not 
killed properly, which leads to many instances running at the same time. 
"""

import os
import subprocess

import rclpy

from audio_bringup.helpers import get_active_nodes, set_param, EXP_DIRNAME, CSV_DIRNAME

# EXPERIMENT = "2020_12_18_stepper"
EXPERIMENT = "2021_02_09_wall"

# FILENAMES = None # set to None to loop through all.
FILENAMES = [
    "motors_nosnr_noprops_sweep_16_newbattery",
    "motors_nosnr_noprops_sweep_16",
    "nomotors_nosnr_noprops_sweep_16_newbattery",
    "nomotors_nosnr_noprops_sweep_16",
    "motors_nosnr_noprops_sweep_15",
    "nomotors_nosnr_noprops_sweep_15",
]


def main(args=None):
    rclpy.init(args=args)

    if FILENAMES is not None:
        filenames = FILENAMES
    else:
        filenames = []
        for (dirpath, dirnames, contents) in os.walk(
            os.path.join(EXP_DIRNAME, EXPERIMENT)
        ):
            if "metadata.yaml" in contents:
                filename = os.path.basename(dirpath)

                # make sure dirpath follows convention.
                assert dirpath == os.path.join(
                    EXP_DIRNAME, EXPERIMENT, filename
                ), dirpath

                filenames.append(filename)
                print("found bag folder:", filename)

    active_nodes = get_active_nodes()
    assert "/csv_writer" in active_nodes

    for filename in filenames:
        print(f"treating {filename}...")
        bag_filename = os.path.join(EXP_DIRNAME, EXPERIMENT, filename)
        csv_filename = os.path.join(
            EXP_DIRNAME, EXPERIMENT, CSV_DIRNAME, filename + ".csv"
        )

        success = False
        while not success:
            print("trying to reset csv_writer...")
            success = set_param("/csv_writer", "filename", "")

        subprocess.run(["ros2", "bag", "play", bag_filename])

        # once bag file playing is done, we continue with below.
        set_param("/csv_writer", "filename", csv_filename)


if __name__ == "__main__":
    main()
