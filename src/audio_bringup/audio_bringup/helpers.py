#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
helpers.py: general launching functions.
"""
import os
import subprocess


LOG_LEVEL = "info"
TOPICS_TO_RECORD =  ['/audio/signals_f', '/geometry/pose_raw', '/crazyflie/status', '/crazyflie/motors']
#TOPICS_TO_RECORD = ['--all'] 
CSV_DIRNAME = "csv_files/"
WAV_DIRNAME = "export/"
EXP_DIRNAME = os.getcwd() + "/experiments/"

def get_filename(**params):
    source_flag = "None" if params.get("source") is None else params.get("source")
    props_flag = "" if params.get("props")==1 else "no"
    snr_flag = "" if params.get("snr")==1 else "no"
    motors = params.get("motors")
    motors_flag = "" if ((type(motors) == str) or (motors > 0)) else "no"
    ending = "" if params.get("degree", 0) == 0 else f"_{params.get('degree')}"
    ending_distance = "" if params.get("distance", None) in [0, None] else f"_{params.get('distance')}"
    ending_distance += params.get("appendix", "")
    fname = f"{motors_flag}motors_{snr_flag}snr_{props_flag}props_{source_flag}{ending}{ending_distance}"
    return fname


def set_param(node_name, param_name, param_value):
    if type(param_value) != str:
        param_value = str(param_value)
    param_pid = subprocess.Popen(['ros2', 'param', 'set', node_name, param_name, param_value], stdout=subprocess.PIPE)
    print('waiting to set params:', param_name, param_value)
    out_bytes, err = param_pid.communicate()
    out_string = out_bytes.decode("utf-8").strip()
    if out_string == "Set parameter successful":
        return True
    else:
        print("set_param error:", out_string)
        return False


def get_launch_description(node_config, log_level=LOG_LEVEL, bag_filename=""):
    import launch
    import launch.actions
    import launch.substitutions
    import launch_ros.actions
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


def get_active_nodes():
    param_pid = subprocess.Popen(['ros2', 'node', 'list'], stdout=subprocess.PIPE)
    out_bytes, err = param_pid.communicate()
    out_string = out_bytes.decode("utf-8").strip()
    return out_string


