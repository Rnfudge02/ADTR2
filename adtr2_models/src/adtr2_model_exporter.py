#!/usr/bin/python3

#Copyright [2024] [Robert Fudge]
#SPDX-FileCopyrightText: © 2024 Robert Fudge <rnfudge@mun.ca>
#SPDX-License-Identifier: {Apache-2.0}

from ultralytics import YOLO
import sys
import os

"""!@package docstring detection.launch.py
Responsible for launching yolo_ros executable to use object detection.

Requires that the devices module be started first.
"""

def model_exporter(model_path, arch, bat = 1):
    """!Handles converting models to the appropriate type.
 
        The script will attempt to export the model with
        optimizations based on architecture.

        @type model_path: str
        @param model_path: String containing the absolute path
        to the YOLO model.

        @type arch: str
        @param arch: String specifying architecture hint for exporting
        optimizations.

        @type bat: int
        @param bat: Integer specifying batch number to export with. Affects
        VRAM usage. Default value is 1, this will mess with the encoding, etc.
        More work is needed to properly support batching
    """
    model = YOLO(model_path)

    int8 = False
    dev = ""

    if arch == "aarch64":
        int8 = True
        dev = "dla:0"

    else:
        int8 = False
        dev = "0"

    model.export(format = "onnx", int8 = int8, device = dev, batch = bat)

    res_1 = model_path.replace(".pt", ".onnx")
    res_2 = model_path.replace("n", "n-" + arch).replace(".pt", ".onnx").replace("PT", "ONNX")
    res_3 = model_path.replace("n", "n-" + arch).replace(".pt", ".engine").replace("PT", "TRT")
    os.rename(res_1, res_2)
    os.system("/usr/src/tensorrt/bin/trtexec --onnx=" + res_2 + " --saveEngine=" + res_3 + " --useSpinWait --threads --useCudaGraph")

if __name__ == "__main__":
    model_exporter(sys.argv[1], sys.argv[2])