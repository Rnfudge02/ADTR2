//Copyright [2024] [Robert Fudge]
//SPDX-FileCopyrightText: © 2024 Robert Fudge <rnfudge@mun.ca>
//SPDX-License-Identifier: {Apache-2.0}

#include <cstdio>

#include "adtr2_base/AUVController.hpp"

using namespace adtr2::controller;

AUVController::AUVController(const rclcpp::NodeOptions & options) : ADTR2Module("auv_controller", options), count_(0) {
    std::chrono::duration t_stat = this->reg_and_ret_ms("t_stat", 1000, "Period the node should use when communicating back to the AUVMonitor.");
    std::chrono::duration t_cont = reg_and_ret_ms("t_cont", 250, "Period the node should use to check for detections.");

    internal_status = 0;

    internal_status_msg = std_msgs::msg::UInt8();
    internal_status_publisher = create_publisher<std_msgs::msg::UInt8>(module_prefix + "status", max_messages);

    internal_status_msg.data = internal_status;
}