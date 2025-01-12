#!/usr/bin/python3

#Copyright [2024] [Robert Fudge]
#SPDX-FileCopyrightText: © 2024 Robert Fudge <rnfudge@mun.ca>
#SPDX-License-Identifier: {Apache-2.0}

from ament_index_python.packages import get_package_share_directory
 
import os
import sys
import shutil
import yaml

from ultralytics import YOLO

dir_path = os.path.dirname(os.path.realpath(__file__))

"""!@package docstring model_exporter.py
Responsible for converting the 

Requires that the devices module be started first.
"""
def generate_stage_1():
    """!Handles launching data acquisition
 
    Uses ros2 bag to store topics listed in
    configuration file.
    """
    #Use only official YOLO names here
    model_name = "yolov8s.pt"
    config_file = os.path.join(get_package_share_directory("automata_deployment_toolkit_ros2"), "config", "settings.yaml")
    model_path = "models".join(dir_path.rsplit("src", 1)) + "/PT/" + model_name

    arch = ""
    extra_args = ""

    #Open yaml file
    with open(config_file, 'r') as file:
        config_arr = yaml.safe_load(file)

        arch = config_arr['init']['arch']
        suffix = model_name[-4]

        model = YOLO(model_path)

        if arch == "arm64":
            extra_args = "--useDLACore=0 --allowGPUFallback"

        else:
            pass

        model.export(format = "onnx", batch = 1)

    #Path to output of above
    os.makedirs(model_path.replace(model_path.split("/", -1)[-1], "").replace("PT", "ONNX"), exist_ok = True)
    os.makedirs(model_path.replace(model_path.split("/", -1)[-1], "").replace("PT", "TRT"), exist_ok = True)

    #Output of the export command
    res1 = model_path.replace(".pt", ".onnx")

    #Create path to ONNX dir and move to ONNX dir
    res2 = res1.replace("PT", "ONNX").replace(suffix + ".", suffix + "-" + arch + ".")

    #Create path to engine destination, and run trtexec
    res3 = res2.replace(".onnx", ".engine").replace("ONNX", "TRT")
    shutil.move(res1, res2)

    #Write the TRT configuration options to a .env file
    f = open(dir_path + "/stage_2.env", "w")
    f.write("ARGS=\"--onnx=" + res2 + " --saveEngine=" + res3 + " --useSpinWait --threads --useCudaGraph --best" + extra_args + "\"")
    f.close()

    return

#Run the function when the script starts
if __name__ == "__main__":
    generate_stage_1()

