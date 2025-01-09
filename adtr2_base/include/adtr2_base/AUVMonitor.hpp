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

            //! Enables launching CNN exporter
            /*!
                Starts exporting process by launching exporter.launch.py. Will choose appropriate architecture and settings,
                and export optimized onnx plans and TensorRT engines.

                @pre System cannot be started

                Precondition -> Postcondition:

                !exporter -> exporter

                success -> !exporter

                If an internal error prevents the postconditions from following the preconditions, the response will indicate failure
                and the message should inform of the reason for failure.

                @param request Not used, part of formatting a std_srvs::Trigger callback.
                @param response Informs caller of status of the request.
                @return void
            */
            void launch_exporter(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

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
                //Update data of messages
                daq_current_msg.data = daq_current_status;
                daq_launcher_msg.data = daq_launcher_status;

                exporter_current_msg.data = exporter_current_status;
                exporter_launcher_msg.data = exporter_launcher_status;
                
                system_current_msg.data = system_current_status;
                system_launcher_msg.data = system_launcher_status;

                //Publish launching status
                daq_launcher_publisher->publish(daq_launcher_msg);
                system_launcher_publisher->publish(system_launcher_msg);

                //Publish updated states
                daq_current_publisher->publish(daq_current_msg);
                system_current_publisher->publish(system_current_msg);
            }

            //! Internal function for publishing status of components.
            /*!
                NOT IMPLEMENTED. Will be responsible for checking subscribed topics for liveliness

                @return void.
            !*/
            void __node_checking_callback() {
                if (system_toggled) {
                    //Check ZED2i status

                    //Check VSLAM status

                    //Check OD status

                    if (daq_toggled) {
                        
                    }

                    //Retrieve component statuses and bitmask into uint32
                    system_current_status = 0b0 | (zed_status << 8) | (vslam_status << 16) | (od_status << 24);
                }
            }

            //Timer members
            rclcpp::TimerBase::SharedPtr status_publishing_timer;
            rclcpp::TimerBase::SharedPtr node_checking_timer;

            //Service members
            rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr daq_service;
            rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr exporter_service;
            rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr system_service;

            //Service statuses
            uint8_t daq_launcher_status;
            uint8_t daq_current_status;

            uint8_t exporter_launcher_status;
            uint8_t exporter_current_status;

            uint8_t system_launcher_status;
            uint32_t system_current_status;

            //Component statuses
            uint8_t zed_status;
            uint8_t vslam_status;
            uint8_t od_status;

            //Messages for publishing statuses
            example_interfaces::msg::UInt8 daq_current_msg;
            example_interfaces::msg::UInt8 daq_launcher_msg;

            example_interfaces::msg::UInt8 exporter_current_msg;
            example_interfaces::msg::UInt8 exporter_launcher_msg;

            example_interfaces::msg::UInt32 system_current_msg;
            example_interfaces::msg::UInt8 system_launcher_msg;

            rclcpp::Publisher<example_interfaces::msg::UInt8>::SharedPtr daq_current_publisher;
            rclcpp::Publisher<example_interfaces::msg::UInt8>::SharedPtr daq_launcher_publisher;

            rclcpp::Publisher<example_interfaces::msg::UInt8>::SharedPtr exporter_current_publisher;
            rclcpp::Publisher<example_interfaces::msg::UInt8>::SharedPtr exporter_launcher_publisher;
            
            rclcpp::Publisher<example_interfaces::msg::UInt32>::SharedPtr system_current_publisher;
            rclcpp::Publisher<example_interfaces::msg::UInt8>::SharedPtr system_launcher_publisher;

            //Launching Information
            pid_t daq_process;
            pid_t exporter_process;
            pid_t system_process;

            //Actor interaction statuses
            bool daq_toggled;
            bool exporter_active;
            bool system_toggled;

            //Count (assuming ensures only one instance can be created?)
            size_t count_;
        };
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(adtr2::monitor::AUVMonitor);