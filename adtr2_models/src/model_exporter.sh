#!/bin/bash

#Copyright [2024] [Robert Fudge]
#SPDX-FileCopyrightText: © 2024 Robert Fudge <rnfudge@mun.ca>
#SPDX-License-Identifier: {Apache-2.0}

#AARDK Container Controller V1.0 - Program developed by Robert Fudge, 2024

#Collect architecture information
ARCH=$(uname -m)

#ASCII escape formatting sequences
RESET="\033[0m"
BOLD="\033[1m"
DIM="\033[2m"
ITALIC="\033[3m"
UNDERLINE="\033[4m"
BLINK="\033[5m"

#ASCII foreground formatting sequences
FG_BLACK="\033[30m"
FG_RED="\033[31m"
FG_GREEN="\033[32m"
FG_YELLOW="\033[33m"
FG_BLUE="\033[34m"
FG_MAGENTA="\033[35m"
FG_CYAN="\033[36m"
FG_WHITE="\033[37m"

#ASCII background formatting sequences
BG_BLACK="\033[40m"
BG_RED="\033[41m"
BG_GREEN="\033[42m"
BG_YELLOW="\033[43m"
BG_BLUE="\033[44m"
BG_MAGENTA="\033[45m"
BG_CYAN="\033[46m"
BG_WHITE="\033[47m"

source /opt/ros/humble/setup.bash
source ${ROS_WS}/install/setup.bash

#Get ADTR2 directory
ADTR2_DIR="${ROS_WS}/src/adtr2/adtr2_models"

#Set the appropriate suffix in the PLAT environment variable
if [ ${ARCH} == "aarch64" ]; then
    PLAT=arm64

elif [ ${ARCH} == "x86_64" ]; then
    PLAT=amd64

else
    echo -e "${FG_YELLOW}[Model Exporter]: ${FG_RED}Invalid Platform! Exiting...${RESET}"
    exit 1
fi

#Create the appropriate onnx file if it doesn't currently exist
if [ ! -f "${ADTR2_DIR}/models/ONNX/yolov8n-${PLAT}.onnx" ] || [ ! -f "${ADTR2_DIR}/src/stage_2.env" ]; then
    echo -e "${FG_YELLOW}[Model Exporter]: Creating .onnx file from PyTorch weights${RESET}"
    python3 ${ADTR2_DIR}/src/model_exporter.py
    source ${ADTR2_DIR}/src/stage_2.env
fi

#Create the trt engine file
if [ ! -f "${ADTR2_DIR}/models/TRT/yolo8n-${PLAT}.engine" ]; then
    echo -e "${FG_YELLOW}[Model Exporter]: Creating .engine file from ONNX file${RESET}"
    /usr/src/tensorrt/bin/trtexec $ARGS
fi
