//Copyright [2024] [Robert Fudge]
//SPDX-FileCopyrightText: © 2024 Robert Fudge <rnfudge@mun.ca>
//SPDX-License-Identifier: {Apache-2.0}

#include <cstdio>

#include "adtr2_base/AUVDirector.hpp"

#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection2_d.hpp"

using namespace adtr2::director;

//! AUVDirector class constructor.
/*!
    Initializes class members for the AUVDirector component and configures
    appropriate timers and publishers

    \return Instance of AUVDirector
!*/
AUVDirector::AUVDirector(const rclcpp::NodeOptions & options) : ADTR2Module("auv_director", options), count_(0) {
    //Handle input parameters
    std::chrono::duration t_stat = this->reg_and_ret_ms("t_stat", 1000, "Period the node should use when communicating back to the AUVMonitor.");
    std::chrono::duration t_det = this->reg_and_ret_ms("t_det", 100, "Period the node should use to check for detections.");
    std::chrono::duration t_dir = this->reg_and_ret_ms("t_dir", 1000, "Period the node should use when generating instructions for AUVController to relay.");

    //Use input parameters to initialize class
    status_timer = this->create_wall_timer(t_stat, std::bind(&AUVDirector::status_timer_callback, this));
    detection_timer = this->create_wall_timer(t_det, std::bind(&AUVDirector::detection_timer_callback, this));
    direction_timer = this->create_wall_timer(t_dir, std::bind(&AUVDirector::direction_timer_callback, this));

    internal_status_msg = std_msgs::msg::UInt8();
    internal_status_publisher = create_publisher<std_msgs::msg::UInt8>(module_prefix + "status", max_messages);

    detection_subscriber = create_subscription<vision_msgs::msg::Detection2DArray>("/detections", 10, std::bind(&AUVDirector::detection_img_callback, this, std::placeholders::_1));
    internal_status_msg.data = internal_status;
}
