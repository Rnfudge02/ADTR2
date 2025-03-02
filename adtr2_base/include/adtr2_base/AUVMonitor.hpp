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

//ROS2 includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_srvs/srv/trigger.hpp"

//ADTR2 includes
#include "adtr2_base/ADTR2Module.hpp"

namespace adtr2 {
    namespace monitor {
        //! AUVMonitor - Controls launching other components, and monitoring their status.
        /*!
            Responsible for launching and monitoring ROS2 nodes. This is accomplished with ROS2 services,
            which allow the user/developer to toggle components, and publishers to alert users/other components
            to the status of all components. The services, when triggered, use syscalls to launch and interact with
            the launched scripts as child processes.
        !*/
        class AUVMonitor : public adtr2::ADTR2Module {
        public:
            //! Creates an AUVMonitor Node.
            /*!
                Initializes class members to a known state and sets up required ROS2 services and publisher topics.
                Returns configured ROS2 class instance to caller.

                @param options Options structure used to pass launch arguments to class.
                @return Instance of AUVMonitor.
            !*/
            AUVMonitor(const rclcpp::NodeOptions& options);

            //! Runs on AUVMonitor node death.
            /*!
                Cleans up after class, freeing memory and triggering the destruction of controlled child processes.

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
                Starts system ros2 packages by calling system.launch.py, which determines it's behavior based on the settings.yaml file in the metapackage.

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
            void __status_publishing_callback();

            //! Internal function for publishing status of components.
            /*!
                NOT IMPLEMENTED. Will be responsible for checking subscribed topics for liveliness

                @return void.
            !*/
            void __node_checking_callback();

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

            uint8_t system_launcher_status;
            uint32_t system_current_status;

            //Component statuses
            uint8_t zed_status;
            uint8_t vslam_status;
            uint8_t od_status;

            //Messages for publishing statuses
            std_msgs::msg::UInt8 daq_current_msg;
            std_msgs::msg::UInt8 daq_launcher_msg;

            std_msgs::msg::UInt32 system_current_msg;
            std_msgs::msg::UInt8 system_launcher_msg;

            //Publishers
            rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr daq_current_publisher;
            rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr daq_launcher_publisher;
            
            rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr system_current_publisher;
            rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr system_launcher_publisher;

            //Launching Information
            pid_t daq_process;
            pid_t system_process;

            //Actor interaction statuses
            bool daq_toggled;
            bool system_toggled;

            //Count (assuming ensures only one instance can be created?)
            size_t count_;
        };
    }
}

//Register as component class
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(adtr2::monitor::AUVMonitor);