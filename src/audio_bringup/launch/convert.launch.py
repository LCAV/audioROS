#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
convert.py:  Convert bag file to csv file. 

This module is deprecated, use topic_writer/convert_all.py instead.

"""

import sys

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

FILENAME = "motors_snr_props_source_45"
LOG_LEVEL = "info"
DIRNAME = "csv_files/"

def generate_launch_description(filename=FILENAME, dirname=DIRNAME, log_level=LOG_LEVEL):
    return launch.LaunchDescription([
        # run topic writer
        launch_ros.actions.Node(
            package="topic_writer",
            node_executable="csv_writer",
            node_name=["csv_writer"],
            parameters=[{"filename":filename, "dirname":dirname}],
            arguments=[(f"--ros-args --log-level {log_level}")],
        ),
        # TODO(FD): find out how to set the parameters correctly from the launch file.  
        # set parameters because above does not work.
        #launch.actions.ExecuteProcess(
        #    cmd=['ros2', 'param', 'set', 'csv_writer', 'filename', filename],
        #),
        #launch.actions.ExecuteProcess(
        #    cmd=['ros2', 'param', 'set', 'csv_writer', 'dirname', dirname],
        #),

        # play bag file 
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'play', filename], output='screen'
        )
    ])


def main(argv=sys.argv[1:]):
    ld = generate_launch_description()
    ls = launch.LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return ls.run()

if __name__ == "__main__":
    main()
