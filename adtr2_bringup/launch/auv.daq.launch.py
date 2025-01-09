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
and qhy5-III nodes. Requires devices to be started.
"""

def generate_launch_description():
    """!Handles launching data acquisition
 
    Uses ros2 bag to store topics listed in
    configuration file.
    """
    config_file = os.path.join(get_package_share_directory("automata_deployment_toolkit_ros2"), "config", "settings.yaml")

    date = datetime.now().strftime("%Y_%m_%d")
    timestamp = datetime.now().strftime("%H_%M_%S")

    topics = []

    #Open yaml file
    with open(config_file, 'r') as file:
        config_arr = yaml.safe_load(file)

        if (config_arr['init']['gnss'] == True and config_arr['storage']['gnss']['enabled'] == True):
            topics.extend(config_arr['storage']['gnss']['topics'])

        if (config_arr['init']['qhy5III'] == True and config_arr['storage']['qhy5III']['enabled'] == True):
            topics.extend(config_arr['storage']['qhy5III']['topics'])

        if (config_arr['init']['zed'] == True and config_arr['storage']['zed']['enabled'] == True):
            topics.extend(config_arr['storage']['zed']['topics'])

        if (config_arr['init']['vslam'] == True and config_arr['storage']['vslam']['enabled'] == True):
            topics.extend(config_arr['storage']['zed']['topics'])

        if (config_arr['init']['od'] == True and config_arr['storage']['od']['enabled'] == True):
            topics.extend(config_arr['storage']['zed']['topics'])

        if (config_arr['init']['sr'] == True and config_arr['storage']['sr']['enabled'] == True):
            topics.extend(config_arr['storage']['zed']['topics'])

        dest = os.path.join(config_arr['storage']['destination'], date, timestamp)

    return launch.LaunchDescription([
        LogInfo(msg = f"Launching recording"),
        ExecuteProcess(cmd = ["ros2", "bag", "record", dest, *topics], output = "screen")
    ])