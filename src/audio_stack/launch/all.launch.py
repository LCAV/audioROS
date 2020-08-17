#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
all.launch.py: 
"""

import os
import pathlib

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

LOG_SEVERITY = 3 #0-debug, 1-info, 2-Warn, 3-Error, 4-Fatal
LOG_LEVEL = 'warn'
node_config = {
    'publisher': [{'n_buffer': 2**8}],
    'processor': [],
    'correlator': []
}
# TODO does not work
for node in node_config.keys():
    node_config[node].append({'log_severity': LOG_SEVERITY})

def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                "node_prefix",
                default_value=[launch.substitutions.EnvironmentVariable("USER"), "_"],
                description="Prefix for node names",
            ),
            *[ 
                launch_ros.actions.Node(
                package="audio_stack",
                node_executable=executable,
                output="screen",
                node_name=[
                    launch.substitutions.LaunchConfiguration("node_prefix"),
                    f"name_{executable}",
                ],
                parameters=params, 
                # TODO deprecated but works
                arguments=[(f'__log_level:={LOG_LEVEL}')])
                # TODO doesn't work
                #arguments=[(f'--ros-args --log-level {str.upper(LOG_LEVEL)}')])
                for executable, params in node_config.items()
            ]
        ]
    )
