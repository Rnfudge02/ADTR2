//Copyright [2024] [Robert Fudge]
//SPDX-FileCopyrightText: © 2024 Robert Fudge <rnfudge@mun.ca>
//SPDX-License-Identifier: {Apache-2.0}

#include <cstdio>

#include <sys/types.h>
#include <sys/wait.h>

#include <stdlib.h>

#include "adtr2_base/AUVMonitor.hpp"

using namespace adtr2::monitor;

AUVMonitor::AUVMonitor(const rclcpp::NodeOptions & options) : ADTR2Module("auv_monitor", options), count_(0) {
    std::chrono::duration t_stat = this->reg_and_ret_ms("t_stat", "1000", "Period the AUVMonitor should monitor the system and publish it's status.");

    //Set up timer for publishing statuses and timer for checking if nodes are active
    status_publishing_timer = this->create_wall_timer(t_stat, std::bind(&AUVMonitor::__status_publishing_callback, this));

    //Initialize process ids to 0
    daq_process = 0;
    system_process = 0;

    //Initialize statuses to 0
    daq_current_status = 0;
    daq_launcher_status = 0;

    system_current_status = 0;
    system_launcher_status = 0;

    //Initialize component statuses to 0
    zed_status = 0;
    vslam_status = 0;
    od_status = 0;

    //Create services for toggling components
    daq_service = create_service<std_srvs::srv::Trigger>(module_prefix + "toggle_daq",
        std::bind(&AUVMonitor::toggle_daq, this, std::placeholders::_1, std::placeholders::_2));

    system_service = create_service<std_srvs::srv::Trigger>(module_prefix + "toggle_system",
        std::bind(&AUVMonitor::toggle_system, this, std::placeholders::_1, std::placeholders::_2));

    //Create publishers for component statuses
    daq_current_publisher = create_publisher<example_interfaces::msg::UInt8>(module_prefix + "status/daq_current", max_messages);
    daq_launcher_publisher = create_publisher<example_interfaces::msg::UInt8>(module_prefix + "status/daq_launcher", max_messages);

    system_current_publisher = create_publisher<example_interfaces::msg::UInt32>(module_prefix + "status/system_current", max_messages);
    system_launcher_publisher = create_publisher<example_interfaces::msg::UInt8>(module_prefix + "status/system_launcher", max_messages);
    
    //Create uint8_t messages for publishing status
    daq_current_msg = example_interfaces::msg::UInt8();
    daq_launcher_msg = example_interfaces::msg::UInt8();

    system_current_msg = example_interfaces::msg::UInt32();
    system_launcher_msg = example_interfaces::msg::UInt8();

    //Initialize message data
    daq_current_msg.data = daq_current_status;
    daq_launcher_msg.data = daq_launcher_status;

    system_launcher_msg.data = system_launcher_status;
    system_current_msg.data = system_current_status;
}

AUVMonitor::~AUVMonitor() {
    //Not Implemented
}

void AUVMonitor::toggle_daq(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Starting data acquisition...");

    //If devices are not initialized
    if (!system_toggled) {
        RCLCPP_WARN(this->get_logger(), "System is not started. Cannot start data acquisition.");
        response->success = false;
        response->message = "Data acquisition not launched.";
        return;
    }

    //If devices are initialized
    else {
        //Turn recording on and notify the user
        if (!daq_toggled) {
            pid_t n = fork();

            if (n == 0) {
                //Use execl to run ros2 launch on data acquisition launch script - in child process
                //NULL acts as sentinel
                int launch_result = execl("/bin/bash", "bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && source /home/${USERNAME}/ros_ws/install/setup.bash | ros2 launch adtr2_bringup daq.launch.py", NULL);

                if (launch_result != -1) {
                    RCLCPP_INFO(this->get_logger(), "Launching data acquistion.");
                }

                else {
                    RCLCPP_ERROR(this->get_logger(), "Could not start data acquisition script.");
                    response->success = false;
                    response->message = "Data acquisition not launched.";
                    daq_launcher_status = 1;
                    daq_toggled = false;
                    exit(1);
                }
            }

            //Store pid, format response, update internal member and return
            else {
                daq_process = n;
                response->success = true;
                response->message = "Data acquisition launched.";
                daq_launcher_status = 255;
                daq_toggled = true;
                return;
            }   
        }

        //Turn recording off and notify the user
        else {
            int stop_result = kill(daq_process, SIGINT);

            if (stop_result != -1) {
                waitpid(daq_process, nullptr, 0);
                RCLCPP_INFO(this->get_logger(), "Data acquisition stopped successfully!");
                response->success = true;
                response->message = "Data acquisition stopped.";
                daq_launcher_status = 0;
                daq_toggled = false;
                return;
            }

            else {
                RCLCPP_WARN(this->get_logger(), "Error: Data acquisition process couldn't be killed, is it running?");
                response->success = false;
                response->message = "Data acquisition not stopped.";
                daq_launcher_status = 2;
                daq_toggled = true;
                exit(1);
            }
        }
    }
}

void AUVMonitor::toggle_system(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    if (!(system_current_status == 1)) {
        RCLCPP_INFO(this->get_logger(), "Starting system launch script...");

        pid_t n = fork();

        if (n == 0) {
            //Use execl to run ros2 launch on nv_vslam launch script - in child process
            int launch_result = execl("/bin/bash", "bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && source /home/${USERNAME}/ros_ws/install/setup.bash | ros2 launch adtr2_bringup auv.system.launch.py", NULL);

            if (launch_result != -1) {
                RCLCPP_INFO(this->get_logger(), "System launch script started successfully");
            }

            else {
                RCLCPP_ERROR(this->get_logger(), "Could not start system launch script.");
                response->success = false;
                response->message = "System not launched.";
                system_launcher_status = 1;
                system_toggled = false;
                exit(1);
            }
        }

        else {
            system_process = n;
            response->success = true;
            response->message = "System started.";
            system_launcher_status = 255;
            system_toggled = true;
            return;
        }
    }

    //If System is initialized
    else {
        //Stop the system
        int stop_result = kill(system_process, SIGINT);

        if (stop_result != -1) {
            waitpid(system_process, nullptr, 0);
            RCLCPP_INFO(this->get_logger(), "System stopped successfully!");
            response->success = true;
            response->message = "System stopped successfully.";
            system_launcher_status = 0;
            system_toggled = false;
            return;
        }

        else {
            RCLCPP_WARN(this->get_logger(), "Error: System process couldn't be killed, is it running?");
            response->success = false;
            response->message = "System not stopped, failed to kill process.";
            system_launcher_status = 2;
            system_toggled = true;
            exit(1);
        }
    }
}