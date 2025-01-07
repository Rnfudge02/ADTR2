#!/usr/bin/python3

#Copyright [2024] [Robert Fudge]
#SPDX-FileCopyrightText: © 2024 Robert Fudge <rnfudge@mun.ca>
#SPDX-License-Identifier: {Apache-2.0}

import launch
from launch.actions import ExecuteProcess, LogInfo
from ament_index_python.packages import get_package_share_directory

import os
from datetime import datetime
import yaml

"""!@package docstring daq.launch.py
Responsible for recording selected topics.

Currently supports GNSS Reciever, ZED2i,
and OHY5-III nodes. Requires devices to be started.
"""

def generate_launch_description():
    """!Handles launching data acquisition
 
    Uses ros2 bag to store topics listed in
    configuration file.
    """
    config_file = os.path.join(get_package_share_directory("automata_deployment_toolkit_ros2"), "config", "settings.yaml")
    model_src_path = os.path.join(get_package_share_directory("adtr2_models"), "src", "adtr2_model_exporter.py")
    model_path = os.path.join(get_package_share_directory("adtr2_models"), "models", "PT", "yolo11n.pt")

    arch = ""

    #Open yaml file
    with open(config_file, 'r') as file:
        config_arr = yaml.safe_load(file)

        arch = config_arr['init']['arch'] 

    return launch.LaunchDescription([
        LogInfo(msg = f"Launching optimizer, may take a few minutes"),
        ExecuteProcess(cmd = ["python3", model_src_path, model_path, arch], output = "screen")
    ])