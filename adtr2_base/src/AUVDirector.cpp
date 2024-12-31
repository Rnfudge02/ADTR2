//Copyright [2024] [Robert Fudge]
//SPDX-FileCopyrightText: © 2024 Robert Fudge <rnfudge@mun.ca>
//SPDX-License-Identifier: {Apache-2.0}

#include <cstdio>

#include "adtr2_base/AUVDirector.hpp"

using namespace std::chrono_literals;
using namespace adtr2::director;

//! AUVDirector class constructor.
/*!
    Initializes class members for the AUVDirector component and configures
    appropriate timers and publishers

    \return Instance of AUVDirector
!*/
AUVDirector::AUVDirector(const rclcpp::NodeOptions & options) : Node("auv_director", options), count_(0) {
    status_timer = this->create_wall_timer(250ms, std::bind(&AUVDirector::status_timer_callback, this));
    direction_timer = this->create_wall_timer(250ms, std::bind(&AUVDirector::direction_timer_callback, this));

    internal_status = 0;

    internal_status_msg = example_interfaces::msg::UInt8();
    internal_status_publisher = create_publisher<example_interfaces::msg::UInt8>(package_prefix + "status", max_messages);

    internal_status_msg.data = internal_status;
}
