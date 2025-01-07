#!/usr/bin/python3

#Copyright [2024] [Robert Fudge]
#SPDX-FileCopyrightText: © 2024 Robert Fudge <rnfudge@mun.ca>
#SPDX-License-Identifier: {Apache-2.0}

from ultralytics import YOLO
import sys
import os
import shutil

"""!@package docstring adtr2_model_exporter.py
Responsible for converting the 

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
    extra_args = ""

    if arch == "arm64":
        extra_args = "--useDLACore=0 --allowGPUFallback"

    else:
        pass

    model.export(format = "onnx", batch = bat)

    #Path to output of above
    res1 = model_path.replace(".pt", ".onnx")
    os.makedirs(model_path.replace(".pt", "").replace("PT", "ONNX"), exist_ok = True)
    os.makedirs(model_path.replace(".pt", "").replace("PT", "TRT"), exist_ok = True)

    #Create path to ONNX dir and move to ONNX dir
    res2 = res1.replace("PT", "ONNX")
    shutil.move(res1, res2)

    #Create path to engine destination, and run trtexec
    res3 = res2.replace(".onnx", ".engine").replace("ONNX", "TRT")
    os.system("/usr/src/tensorrt/bin/trtexec --onnx=" + res2 + " --saveEngine=" + res3 + " --useSpinWait --threads --useCudaGraph --best " + extra_args)

if __name__ == "__main__":
    model_exporter(sys.argv[1], sys.argv[2])