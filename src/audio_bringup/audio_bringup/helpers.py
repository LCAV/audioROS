#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
helpers.py: general launching functions.
"""

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

LOG_LEVEL = "warn"


def get_launch_description(node_config, log_level=LOG_LEVEL):
    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                "node_prefix",
                default_value=[launch.substitutions.EnvironmentVariable("USER"), "_"],
                description="Prefix for node names",
            ),
            *[
                launch_ros.actions.Node(
                    package=dict_["pkg"],
                    node_executable=executable,
                    output="screen",
                    node_name=[
                        launch.substitutions.LaunchConfiguration("node_prefix"),
                        f"name_{executable}",
                    ],
                    parameters=dict_.get("params", []),
                    # TODO not deprecated but doesn't work
                    # arguments=[(f'--ros-args --log-level {str.upper(log_level)}')])
                    # TODO deprecated but works
                    arguments=[(f"__log_level:={log_level}")],
                )
                for executable, dict_ in node_config.items()
            ],
        ]
    )

