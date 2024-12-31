#!/usr/bin/python3

#Copyright [2024] [Robert Fudge]
#SPDX-FileCopyrightText: © 2024 Robert Fudge <rnfudge@mun.ca>
#SPDX-License-Identifier: {Apache-2.0}

import os
import yaml

import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

"""!@package docstring monitor.launch.py
Responsible for launching the AUV Monitor and Foxglove Bridge.

The AUV Monitor is responsible for ensuring all launched
devices are behaving as expected.

Foxglove bridge allows vizualization at both a
local and network level with Foxglove Studio.
"""

def generate_launch_description():
    """!Handles launching the AUV Monitor and Foxglove Bridge
 
    Formats Nodes to launch the AUV Monitor, which will
    check the dfiagnostic array to determine the best course of
    action, and Foxglove Bridge, which 
    """

    exec_nodes = []

    config_file = os.path.join(get_package_share_directory("automata_deployment_toolkit_ros2"), "config", "settings.yaml")

    #Open config file
    with open(config_file, 'r') as file:
        #Perform a safe load (User input)
        config_arr = yaml.safe_load(file)

        accel_nodes = []
        exec_nodes = []

        if (config_arr['init']['monitor'] == True):
            #System Monitor Composable Node
            monitor_node = ComposableNode(
                package = 'adtr2_base',
                plugin = 'adtr2::monitor::AUVMonitor',
                name = 'auv_monitor_node',
                extra_arguments = [{'use_intra_process_comms': True}]
            )

            accel_nodes.append(monitor_node)

        if (config_arr['init']['foxglove'] == True):
            #Foxglove Composable Node
            foxglove_node = ComposableNode(
                package = 'foxglove_bridge',
                plugin = 'foxglove_bridge::FoxgloveBridge',
                name = 'foxglove_node',
                parameters = [{
                    "port" : 8765,
                    "address" : "0.0.0.0",
                    "tls" : False,
                    "min_qos_depth" : 1,
                    "max_qos_depth" : 10,
                    "num_threads" : config_arr['config']['foxglove']['num_threads'],
                    "send_buffer_limit" : config_arr['config']['foxglove']['buff_size'] * 1024000,
                    "use_sim_time" : False,
                    "capabilities" : ["clientPublish", "parameters", "parametersSubscribe", "services", "connectionGraph", "assets"],
                    "include_hidden" : False
                }],
                extra_arguments = [{'use_intra_process_comms': True}]
            )

            accel_nodes.append(foxglove_node)

        accel_node_container = ComposableNodeContainer(
            package = "rclcpp_components",
            name = "monitor_container",
            namespace = "",
            executable = "component_container_mt",
            composable_node_descriptions = accel_nodes,
            output = "screen"
        )

    return launch.LaunchDescription([accel_node_container] + exec_nodes)