//Copyright [2025] [Robert Fudge]
//SPDX-FileCopyrightText: © 2025 Robert Fudge <rnfudge@mun.ca>
//SPDX-License-Identifier: Apache-2.0
#pragma once

//
#include "rclcpp/rclcpp.hpp"

//
#include <chrono>
#include <string>

namespace adtr2 {
    //! ADTR2Module - Contains common code and utilities for ADTR2 modules.
    /*!
        Not mean't to be instantiated by the user, hence the protected constructor.
        Responsible for setting up debugging, max messages, and provides common utilities.
    !*/
    class ADTR2Module : public rclcpp::Node {
    public:
        ~ADTR2Module() {

        }

    protected:
        //! ADTR2Module - constructor for the base class of all ADTR2 modules.
        /*!
            Will run prior to the derived classes constructor, if there's a parameter common to all
            modules, it likely belongs here.
        !*/
        ADTR2Module(std::string name, const rclcpp::NodeOptions & options) : Node(name, options) {
            //Format descriptions for common arguments
            auto debug_desc = rcl_interfaces::msg::ParameterDescriptor{};
            debug_desc.description = "Debug flag, accepts 1 to enable debugging or 0 to disable.";

            auto max_messages_desc = rcl_interfaces::msg::ParameterDescriptor{};
            debug_desc.description = "The number of max messages the class will interact with. Default is 10. Larger values may cause instability.";

            //Declare paramater for debugging and retrieve as bool
            this->declare_parameter("debug", false, debug_desc);
            debug_flag = this->get_parameter("debug").as_bool();

            //Declare paramater for max message count
            this->declare_parameter("max_messages", 10, max_messages_desc);
            max_messages = this->get_parameter("max_messages").as_int();

            //If debugging is enabled alert the user
            if (debug_flag) {
                RCLCPP_INFO(this->get_logger(), "Debugging mode enabled.");
            }

            module_prefix = name;
        }

        //! Creates an AUVMonitor Node.
        /*!
            Initializes class members to a known state and sets up required ROS2 services and publisher topics.
            @param stat_update_int Interval at which to publish topic updates. TEMP DISABLED
            @param node_check_int Interval at which to check node topics. TEMP DISABLED
            @return Instance of AUVMonitor.
        !*/
        std::chrono::milliseconds reg_and_ret_ms(std::string param_name, uint64_t default_val = 100, std::string param_desc = "Description not available.") {
            //Format description
            auto desc = rcl_interfaces::msg::ParameterDescriptor{};
            desc.description = param_desc;

            //Declare paramater and retrieve value as int
            this->declare_parameter(param_name, default_val, desc);
            int64_t param_int = this->get_parameter(param_name).as_int();

            //Convert to milliseconds and return to caller
            std::chrono::milliseconds t_param(param_int);
            return t_param;
        }

        //Common class variables
        bool debug_flag;
        std::string module_prefix;
        int max_messages;

    private:

    };
}