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
        //! AUVController - Controls interactions with MAVROS for the AUV.
        /*!
            Responsible for recieving data from the AUVDirector, and sending appropriate waypoints to mavros.
        !*/
        class AUVController : public adtr2::ADTR2Module {
        public:
            //! Creates an AUVController Node.
            /*!
                Initializes class members to a known state and sets up required ROS2 services for interacting with AUVDirector
                and MAVROS node.

                @param options Options structure used to pass launch arguments to class.
                @return Instance of AUVController.
            !*/
            AUVController(const rclcpp::NodeOptions& options);

        protected:

        private:
            //! Callback for sending messages to MAVROS.
            /*!

                @return None.
            !*/
            void controller_timer_callback() {
        
                return;
            }

            //! Callback for updating device heartbeat.
            /*!

                @return None.
            !*/
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