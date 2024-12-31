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

namespace adtr2 {
    namespace director {
        const std::string package_prefix = "/auv_director/";
        const int max_messages = 10;

        //! AUVDirector - Controls the device based on environmental information.
        /*!
            Responsible for interacting with the perceptor component to understand the environment
            and interacting with the controller component for navigating the submersible
        !*/
        class AUVDirector : public rclcpp::Node {
        public:
            AUVDirector(const rclcpp::NodeOptions & options);

        protected:

        private:
            void direction_timer_callback() {
        
                return;
            }

            void status_timer_callback() {
        
                internal_status_publisher->publish(internal_status_msg);
            }

            uint8_t internal_status;

            example_interfaces::msg::UInt8 internal_status_msg;

            rclcpp::TimerBase::SharedPtr direction_timer;
            rclcpp::TimerBase::SharedPtr status_timer;
            rclcpp::Publisher<example_interfaces::msg::UInt8>::SharedPtr internal_status_publisher;

            //Assuming is used to allow only one instance of the class to exist
            size_t count_;
        };
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(adtr2::director::AUVDirector);