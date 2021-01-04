#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
helpers.py: general launching functions.
"""

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

LOG_LEVEL = "info"

TOPICS_TO_RECORD =  ['/audio/signals_f', '/geometry/pose_raw']

def get_launch_description(node_config, log_level=LOG_LEVEL, bag_filename=""):
    logger = launch.substitutions.LaunchConfiguration("log_level")
    launch_arguments = [
        launch.actions.DeclareLaunchArgument(
            "log_level",
            default_value=[log_level],
            description="Logging level",
        )
    ]

    if bag_filename != "":
        launch_arguments.append(
            launch.actions.ExecuteProcess(
                cmd=['ros2', 'bag', 'record', '-o', bag_filename] + TOPICS_TO_RECORD,
                output='screen'
                )
        )
    launch_arguments += [
        launch_ros.actions.Node(
            package=dict_["pkg"],
            node_executable=executable,
            output="screen",
            parameters=dict_.get("params", []),
            arguments=['--ros-args', '--log-level', logger],
        )
        for executable, dict_ in node_config.items()
    ]
    return launch.LaunchDescription(launch_arguments)
