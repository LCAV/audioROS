#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
all.launch.py: 
"""

import os
import pathlib
import sys

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

LOG_LEVEL = 'warn'

node_config = {
    'crazyflie': {
        'params': [],
        'pkg': 'audio_setup'
    },
    'file_publisher': {
        'params': [{'n_buffer': 2**8, 'publish_rate': 100}],
        'pkg': 'audio_publisher'
    },
    'processor': {
        'params': [{'bf_method':'mvdr'}],
        'pkg': 'audio_stack'
    },
    'correlator': {
        'params': [{'noise':'', 'frequency':'', 'window':'tukey'],
        'pkg': 'audio_stack'
    }
}

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
                parameters=dict_['params'], 
                # TODO deprecated but works
                arguments=[(f'__log_level:={LOG_LEVEL}')])
                # TODO not deprecated but doesn't work
                #arguments=[(f'--ros-args --log-level {str.upper(LOG_LEVEL)}')])
                for executable, dict_ in node_config.items()
            ]
        ]
    )

def main(argv=sys.argv[1:]):
    print('launching with', argv)
    ld = generate_launch_description()
    ls = launch.LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return ls.run()

if __name__ == '__main__':
    main()
