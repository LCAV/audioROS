#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
helpers.py: general launching functions.
"""
import os
import subprocess
import yaml

LOG_LEVEL = "info"
BAG_RECORD_ROOT = ""
BAG_PLAYBACK = ""
TOPICS_TO_RECORD = [
    "/audio/signals_f",
    "/geometry/pose_raw",
    "/crazyflie/status",
    "/crazyflie/motors",
]
# TOPICS_TO_RECORD = ['--all']
CSV_DIRNAME = "csv_files/"
WAV_DIRNAME = "export/"
EXP_DIRNAME = os.getcwd() + "/datasets/"


def get_filename(**params):
    source_flag = "None" if params.get("source") is None else params.get("source")
    props_flag = "" if params.get("props") == 1 else "no"
    snr_flag = params.get("bin_selection", 0)
    motors = params.get("motors")
    motors_flag = "" if ((type(motors) == str) or (motors > 0)) else "no"
    ending = "" if params.get("degree", 0) == 0 else f"_{params.get('degree')}"
    ending_distance = (
        ""
        if params.get("distance", None) in [0, None]
        else f"_{params.get('distance')}"
    )
    ending_distance += params.get("appendix", "")
    fname = f"{motors_flag}motors_binsel{snr_flag}_{props_flag}props_{source_flag}{ending}{ending_distance}"
    return fname


def set_param(node_name, param_name, param_value):
    if type(param_value) != str:
        param_value = str(param_value)
    param_pid = subprocess.Popen(
        ["ros2", "param", "set", node_name, param_name, param_value],
        stdout=subprocess.PIPE,
    )
    print("waiting to set params:", param_name, param_value)
    out_bytes, err = param_pid.communicate()
    out_string = out_bytes.decode("utf-8").strip()
    if out_string == "Set parameter successful":
        return True
    else:
        print("set_param error:", out_string)
        return False


def get_launch_description(
    node_config, log_level=LOG_LEVEL, bag_record_root="", bag_playback=""
):
    import launch
    import launch.actions
    import launch.substitutions
    import launch_ros.actions

    bag_record = ""
    if bag_record_root != "":
        for counter in range(100):
            bag_record = f"{bag_record_root}{counter}"
            if not os.path.exists(bag_record):
                break

    logger = launch.substitutions.LaunchConfiguration("log_level")
    launch_arguments = [
        launch.actions.DeclareLaunchArgument(
            "log_level", default_value=[log_level], description="Logging level",
        )
    ]

    if bag_record != "":
        print("adding rosbag record process")
        launch_arguments.append(
            launch.actions.ExecuteProcess(
                cmd=["ros2 bag record -o", bag_record] + TOPICS_TO_RECORD, shell=True
            )
        )
    if bag_playback != "":
        print("adding rosbag play process")
        launch_arguments.append(
            launch.actions.ExecuteProcess(
                cmd=["ros2 bag play", bag_playback], shell=True
            )
        )
    for executable, dict_ in node_config.items():
        if "params" in dict_.keys():
            raise DeprecationWarning("Do not use params, but ros__parameters")

        launch_arguments.append(
            launch_ros.actions.Node(
                package=dict_["pkg"],
                executable=executable,
                output="screen",
                parameters=dict_.get("ros__parameters", []),
                arguments=["--ros-args", "--log-level", logger],
            )
        )
    return launch.LaunchDescription(launch_arguments)


def parse_params_file(params_file):
    launch_options_dict = dict(
        log_level=LOG_LEVEL, bag_playback=BAG_PLAYBACK, bag_record_root=BAG_RECORD_ROOT
    )
    with open(params_file) as f:
        full_config_dict = yaml.load(f, Loader=yaml.FullLoader)
        launch_config_dict = full_config_dict.get("launch", {}).get(
            "ros__parameters", {}
        )
        launch_options_dict.update(launch_config_dict)
    return launch_options_dict


def get_active_nodes():
    param_pid = subprocess.Popen(["ros2", "node", "list"], stdout=subprocess.PIPE)
    out_bytes, err = param_pid.communicate()
    out_string = out_bytes.decode("utf-8").strip()
    return out_string
