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

#include "adtr2_base/ADTR2Module.hpp"

#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection2_d.hpp"

namespace adtr2 {
    namespace director {
        //! AUVDirector - Controls the device based on environmental information.
        /*!
            Responsible for interacting with the perceptor component to understand the environment
            and interacting with the controller component for navigating the submersible.
        !*/
        class AUVDirector : public ADTR2Module {
        public:
            AUVDirector(const rclcpp::NodeOptions& options);

        protected:

        private:
            void detection_timer_callback() {

                return;
            }
            void direction_timer_callback() {
        
                return;
            }

            void status_timer_callback() {
                internal_status_msg.data = internal_status;
                internal_status_publisher->publish(internal_status_msg);
            }

            void detection_img_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "Received a Detection2DArray message with %zu detections", msg->detections.size());
            }

            uint8_t internal_status;

            example_interfaces::msg::UInt8 internal_status_msg;

            rclcpp::TimerBase::SharedPtr detection_timer;
            rclcpp::TimerBase::SharedPtr direction_timer;
            rclcpp::TimerBase::SharedPtr status_timer;
            rclcpp::Publisher<example_interfaces::msg::UInt8>::SharedPtr internal_status_publisher;

            uint8_t quadrant_arr;

            rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_subscriber;

            example_interfaces::msg::UInt8 quadrant_arr_msg;
            rclcpp::Publisher<example_interfaces::msg::UInt8>::SharedPtr quadrant_to_nav_publisher;

            //Assuming is used to allow only one instance of the class to exist
            size_t count_;
        };
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(adtr2::director::AUVDirector);