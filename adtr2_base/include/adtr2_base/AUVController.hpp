//Copyright [2024] [Robert Fudge]
//SPDX-FileCopyrightText: © 2024 Robert Fudge <rnfudge@mun.ca>
//SPDX-License-Identifier: Apache-2.0
#pragma once

//Standard library includes
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <cstdlib>
#include <unistd.h>

#include <sys/types.h>
#include <sys/wait.h>

//RCLCPP includes
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/u_int8.hpp"
#include "std_srvs/srv/trigger.hpp"

//ADTR2 base class
#include "adtr2_base/ADTR2Module.hpp"

namespace adtr2 {
    namespace controller {
        class AUVController : public adtr2::ADTR2Module {
        public:
            AUVController(const rclcpp::NodeOptions & options);

        protected:

        private:
            void controller_timer_callback() {
        
                return;
            }

            void status_timer_callback() {
        
                internal_status_publisher->publish(internal_status_msg);
            }

            uint8_t internal_status;

            example_interfaces::msg::UInt8 internal_status_msg;

            rclcpp::TimerBase::SharedPtr controller_timer;
            rclcpp::TimerBase::SharedPtr status_timer;
            rclcpp::Publisher<example_interfaces::msg::UInt8>::SharedPtr internal_status_publisher;

            size_t count_;
        };
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(adtr2::controller::AUVController);