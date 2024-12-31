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
#include "example_interfaces/msg/u_int32.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace adtr2 {
    namespace monitor {
        //Class constants
        const std::string package_prefix = "/auv_monitor/";
        const int max_messages = 10;

        //! AUVMonitor - Controls launching other components, and monitoring their status
        /*!
            Responsible for launching and monitoring ROS2 nodes. This is accomplished with ROS2 services,
            which allow the user/developer to toggle components, and publishers to alert users/other components
            to the status of all components.
        !*/
        class AUVMonitor : public rclcpp::Node {
        public:
            //! Creates an AUVMonitor Node.
            /*!
                Initializes class members to a known state and sets up required ROS2 services and publisher topics.
                @param stat_update_int Interval at which to publish topic updates. TEMP DISABLED
                @param node_check_int Interval at which to check node topics. TEMP DISABLED
                @return Instance of AUVMonitor.
            !*/
            AUVMonitor(const rclcpp::NodeOptions & options);

            //! Runs on AUVMonitor node death.
            /*!
                Cleans up after class, freeing memory and triggering the destruction of controlled instances.
                @return None.
            !*/
            ~AUVMonitor();

        protected:
            //! Enables toggling of data acqusition.
            /*!
                Checks internal status members to determine the state of the system. If the appropriate preconditions are met,
                data acquisition will start.

                @pre System must be started

                Precondition -> Postcondition:

                !system -> Nothing Occurs

                (system) && !daq -> daq

                (system) && daq -> !daq

                If an internal error prevents the postconditions from following the preconditions, the response will indicate failure
                and the message should inform of the reason for failure.

                @param request Not used, part of formatting a std_srvs::Trigger callback.
                @param response Informs caller of status of the request.
                @return void
            */
            void toggle_daq(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

            //! Enables toggling system components
            /*!
                Starts system parameters by calling system.launch.py, which determines it's behavior

                @pre None

                Precondition -> Postcondition:

                !system -> system

                system -> !system

                If an internal error prevents the postconditions from following the preconditions, the response will indicate failure
                and the message should inform of the reason for failure.

                @param request Not used, part of formatting a std_srvs::Trigger callback.
                @param response Informs caller of status of the request.
                @return void
            */
            void toggle_system(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        private:
            //! Internal function for publishing status of components.
            /*!
                Updates the data of status messages, and then publishes them
                at the time interval specified in the constructor.

                @return void
            */
            void __status_publishing_callback() {
                uint32_t temp_sys_msg = 0;

                //Update data of messages
                daq_status_msg.data = daq_status;

                temp_sys_msg = system_launcher_status | (zed_status << 8) | (vslam_status << 16) | (od_status << 24);
                
                system_status_msg.data = temp_sys_msg;

                //Publish updated messages
                daq_stat_publisher->publish(daq_status_msg);
                system_stat_publisher->publish(system_status_msg);
            }

            //! Internal function for publishing status of components.
            /*!
                NOT IMPLEMENTED. Will be responsible for checking subscribed topics for liveliness

                @return void.
            !*/
            void __node_checking_callback() {
                return;
            }

            //Timer members
            rclcpp::TimerBase::SharedPtr status_publishing_timer;
            rclcpp::TimerBase::SharedPtr node_checking_timer;

            //Service members
            rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr daq_service;
            rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr system_service;

            //Component statuses
            uint8_t daq_status;

            uint8_t system_launcher_status;
            uint8_t zed_status;
            uint8_t vslam_status;
            uint8_t od_status;
            uint32_t system_status;

            //Messages for publishing status
            example_interfaces::msg::UInt8 daq_status_msg;
            example_interfaces::msg::UInt32 system_status_msg;

            rclcpp::Publisher<example_interfaces::msg::UInt8>::SharedPtr daq_stat_publisher;
            rclcpp::Publisher<example_interfaces::msg::UInt32>::SharedPtr system_stat_publisher;

            //Launching Information
            pid_t daq_process;
            pid_t system_process;

            bool daq_on;
            bool system_on;

            //Count (assuming ensures only one instance can be created?)
            size_t count_;
        };
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(adtr2::monitor::AUVMonitor);
