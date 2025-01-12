#!/usr/bin/python3

#Copyright [2024] [Robert Fudge]
#SPDX-FileCopyrightText: © 2024 Robert Fudge <rnfudge@mun.ca>
#SPDX-License-Identifier: {Apache-2.0}

#Import system modules
import os
import sys
from typing import Any, Dict
import yaml

#Import ament modules
from ament_index_python import get_package_share_directory

#Import ROS2 launch modules
import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    #Set up absolute paths to files
    config_file = os.path.join(get_package_share_directory("automata_deployment_toolkit_ros2"), "config", "settings.yaml")
    
    gnss_config_file = os.path.join(get_package_share_directory("automata_deployment_toolkit_ros2"), "config", "nmea_serial_driver.yaml")
    usb_cam_config_file = os.path.join(get_package_share_directory("automata_deployment_toolkit_ros2"), "config", "qhy_params.yaml")

    #ZED configuration files stored by package
    common_config = os.path.join(get_package_share_directory("automata_deployment_toolkit_ros2"), "config", "zed", "common.yaml")
    camera_config = os.path.join(get_package_share_directory("automata_deployment_toolkit_ros2"), "config", "zed", "zed2i.yaml")
    ffmpeg_config = os.path.join(get_package_share_directory("automata_deployment_toolkit_ros2"), "config", "zed", "ffmpeg.yaml")
    xacro_path = os.path.join(get_package_share_directory("zed_wrapper"), "urdf", "zed_descr.urdf.xacro")

    #Lists for containerized and non-containerized node
    accel_nodes = []
    exec_nodes = []

    #Open config file
    with open(config_file, 'r') as file:
        #Perform a safe load (User input)
        config_arr = yaml.safe_load(file)

        #Copy commonly used parameters to local variables
        zed_model = config_arr['config']['zed']['model']
        zed_w = config_arr['config']['zed']['width']
        zed_h = config_arr['config']['zed']['height']

        #Initialize ZED camera
        if (config_arr['init']['zed'] == True):
            #Create the ZED composable node - USe IPC for speedup
            zed_node = ComposableNode(
                package ="zed_components",
                plugin ="stereolabs::ZedCamera",
                name ="zed_node",
                namespace = zed_model,
                parameters = [
                    common_config, camera_config, ffmpeg_config
                ],
                remappings = [
                    ("zed_node/left/camera_info", "/left/camera_info_rect"),
                    ("zed_node/right/camera_info", "/right/camera_info_rect"),
                ],
                extra_arguments = [{"use_intra_process_comms": True}]
            )

            #Formats the left rectified image into rgb8 and publishes it with camera info (NITROS-enabled)
            image_format_converter_node_left = ComposableNode(
                package = "isaac_ros_image_proc",
                plugin = "nvidia::isaac_ros::image_proc::ImageFormatConverterNode",
                name = "image_format_node_left",
                parameters = [{
                    "encoding_desired": "rgb8",
                    "image_width": zed_w,
                    "image_height": zed_h,
                }],
                remappings = [
                    ("image_raw", "/" + zed_model + "/zed_node/left/image_rect_color"),
                    ("image", "left/image_rect_mono")
                ]
            )

            #Formats the right rectified image into rgb8 and publishes it with camera info (NITROS-enabled)
            image_format_converter_node_right = ComposableNode(
                package = "isaac_ros_image_proc",
                plugin = "nvidia::isaac_ros::image_proc::ImageFormatConverterNode",
                name = "image_format_node_right",
                parameters = [{
                    "encoding_desired": "rgb8",
                    "image_width": zed_w,
                    "image_height": zed_h,
                }],
                remappings = [
                    ("image_raw", "/" + zed_model + "/zed_node/right/image_rect_color"),
                    ("image", "right/image_rect_mono")
                ]
            )

            #Robot-State Publisher Composable node
            rsp_node = ComposableNode(
                package = "robot_state_publisher",
                plugin = "robot_state_publisher::RobotStatePublisher",
                name = "robot_state_publisher",
                parameters = [{
                    "robot_description": Command([
                        "xacro", " ", xacro_path, ' ',
                        "camera_name:=", zed_model, ' ',
                        "camera_model:=", zed_model
                    ])
                }]
            )

            #Add nodes to appropriate lists
            accel_nodes.append(zed_node)
            accel_nodes.append(image_format_converter_node_left)
            accel_nodes.append(image_format_converter_node_right)
            accel_nodes.append(rsp_node)

        if (config_arr['init']['static_tf'] == True):
            #Publishes static transform,
            static_tf_publisher = ComposableNode(
                name = "static_transform_publisher",
                package = "tf2_ros",
                plugin = "tf2_ros::StaticTransformBroadcasterNode",
                parameters = [{
                    "frame_id": "base_link",
                    "parent_frame_id": "map",
                    "translation.x": 0.0,
                    "translation.y": 0.0,
                    "translation.z": 0.1,
                    "rotation.x": -0.5,
                    "rotation.y": 0.5,
                    "rotation.z": -0.5,
                    "rotation.w": 0.5
                }]
            )

            accel_nodes.append(static_tf_publisher)

        #Initialize visual SLAM
        if (config_arr['init']['vslam'] == True):
            needs_denoising = False

            use_viz = config_arr['config']['vslam']['viz']

            fuse_imu = False
            gynd = 0.0
            gyrw = 0.0
            acnd = 0.0
            acrw = 0.0
            imu_freq = 0.0
            jitter_thresh = 0.0

            base_frame = ""
            imu_frame = ""

            optical_frames = []
            remap = []

            num_cam = 0

            if (config_arr['init']['qhy5III'] == True):
                needs_denoising = True

                base_frame = "qhy"
                
                fuse_imu = False

            if (config_arr['init']['zed'] == True):
                needs_denoising = False

                fuse_imu = True
                gynd = 0.000244
                gyrw = 0.000019393
                acnd = 0.001862
                acrw = 0.003
                imu_freq = config_arr['config']['zed']['imu_hz']
                jitter_thresh = 35.00

                base_frame = zed_model + "_camera_center"
                imu_frame = zed_model + "_imu_link"

                optical_frames.append(zed_model + "_left_camera_optical_frame")
                optical_frames.append(zed_model + "_right_camera_optical_frame")

                num_cam += 1

                remap = [
                    ("/visual_slam/image_" + str(num_cam - 1), "left/image_rect_mono"),
                    ("/visual_slam/camera_info_"  + str(num_cam - 1), "left/camera_info_rect"),
                    ("/visual_slam/image_"  + str(num_cam), "right/image_rect_mono"),
                    ("/visual_slam/camera_info_"  + str(num_cam), "right/camera_info_rect"),
                    ("/visual_slam/imu", zed_model + "/zed_node/imu/data")
                ]

            #Visual SLAM composable node (working to do multiple cameras) (NITROS-enabled)
            visual_slam_node = ComposableNode(
                name = "visual_slam_node",
                package = "isaac_ros_visual_slam",
                plugin = "nvidia::isaac_ros::visual_slam::VisualSlamNode",
                parameters = [{
                    "enable_localization_n_mapping": True,
                    "denoise_input_images": needs_denoising,
                    "rectified_images": True,
                    "enable_debug_mode": False,
                    "verbosity": 1,
                    "debug_dump_path": "/tmp/cuvslam",
                    "enable_slam_visualization": use_viz,
                    "enable_landmarks_view": use_viz,
                    "enable_observations_view": use_viz,
                    "map_frame": "map",
                    "odom_frame": "odom",
                    "base_frame": base_frame,
                    "imu_frame": imu_frame,
                    "camera_optical_frames": optical_frames,
                    "enable_imu_fusion": fuse_imu,
                    "gyro_noise_density": gynd,
                    "gyro_random_walk": gyrw,
                    "accel_noise_density": acnd,
                    "accel_random_walk": acrw,
                    "calibration_frequency": imu_freq,
                    "img_jitter_threshold_ms": jitter_thresh,
                    "enable_planar_mode": False,
                }],
                remappings = remap
            )

            accel_nodes.append(visual_slam_node)

        if (config_arr['init']['od'] == True):
            model_file = os.path.join("/home/auv-deployment/ros_ws/src/adtr2/adtr2_models", "models", "ONNX", "yolov8s-" + config_arr['init']['arch'] + ".onnx")
            engine_file = os.path.join("/home/auv-deployment/ros_ws/src/adtr2/adtr2_models", "models", "TRT", "yolov8s-" + config_arr['init']['arch'] + ".engine")

            #Encoder node should default output nitros_list_nchw_rgb_f32
            encoder_node = ComposableNode(
                name="dnn_encoder",
                    package="isaac_ros_dnn_image_encoder",
                    plugin="nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode",
                    parameters=[{
                        "input_image_width": zed_w,
                        "input_image_height": zed_h,
                        "network_image_width": 640,
                        "network_image_height": 640,
                        "image_mean": [0.5, 0.5, 0.5],
                        "image_stddev": [0.5, 0.5, 0.5],
                        "input_encoding": "rgb8",
                        "camera_info_input_topic": "left/camera_info_rect",
                    }],
                    remappings = [
                        ("image", "/left/image_rect_mono"),
                        ("encoded_tensor", "tensor_pub")
                    ]
            )

            tensor_rt_node = ComposableNode(
                name="tensor_rt",
                package="isaac_ros_tensor_rt",
                plugin="nvidia::isaac_ros::dnn_inference::TensorRTNode",
                parameters=[{
                    "model_file_path": model_file,
                    "engine_file_path": engine_file,
                    'input_binding_names': ["images"],
                    'output_binding_names': ["output0"],
                    "input_tensor_names": ["input_tensor"],
                    "output_tensor_names": ["output_tensor"],
                    "input_tensor_formats": ["nitros_tensor_list_nchw_rgb_f32"],
                    "output_tensor_formats": ["nitros_tensor_list_nchw_rgb_f32"],
                    "verbose": True,
                    "force_engine_update": False
                }]
            )

            yolov8_decoder_node = ComposableNode(
                name="yolov8_decoder_node",
                package="isaac_ros_yolov8",
                plugin="nvidia::isaac_ros::yolov8::YoloV8DecoderNode",
                parameters=[{
                    "confidence_threshold": 0.40,
                    "nms_threshold": 0.45,
                }]
            )

            if (config_arr['config']['od']['viz'] == True):
                viz_resizer = ComposableNode(
                    name='resize_node',
                    package='isaac_ros_image_proc',
                    plugin='nvidia::isaac_ros::image_proc::ResizeNode',
                    parameters=[{
                        'input_qos': "DEFAULT",
                        'output_width': 640,
                        'output_height': 640,
                        'num_blocks': 40,
                        'keep_aspect_ratio': False,
                        'encoding_desired': "rgb8",
                    }],
                    remappings=[
                        ("image", "/left/image_rect_mono"),
                        ('camera_info', "left/camera_info_rect"),
                    ],
                    )
                
                accel_nodes.append(viz_resizer)

                viz_node = Node(
                    package='isaac_ros_yolov8',
                    executable='isaac_ros_yolov8_visualizer.py',
                    name='yolov11_visualizer',
                    remappings = [
                        ("/yolov8_encoder/resize/image", "resize/image"),
                        ("/yolov8_processed_image", "/yolo11_processed_image")
                    ]
                )

                exec_nodes.append(viz_node)

            accel_nodes.append(encoder_node)
            accel_nodes.append(tensor_rt_node)
            accel_nodes.append(yolov8_decoder_node)

        #If gnss is true, add the file to a LaunchDescription
        if config_arr['init']['gnss'] == True:
            gnss_node = Node(
                package="nmea_navsat_driver",
                executable="nmea_serial_driver",
                output="screen",
                parameters=[gnss_config_file]
            )
            
            exec_nodes.append(gnss_node)

        #If qhy5III is true, add config to a Node
        if config_arr['init']['qhy5III'] == True:
            qhy5III_node = ComposableNode(
                package = "usb_cam",
                plugin = "usb_cam_node_exe",
                namespace = "/qhy5III",
                parameters = [usb_cam_config_file]
            )
            
            accel_nodes.append(qhy5III_node)

        nv_accel_container = ComposableNodeContainer(
            package = "rclcpp_components",
            name = "zed_container",
            namespace = "",
            executable = "component_container_mt",
            composable_node_descriptions = accel_nodes,
            output = "screen"
        )
        
    return launch.LaunchDescription([nv_accel_container] + exec_nodes)