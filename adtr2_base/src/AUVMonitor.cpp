//Copyright [2024] [Robert Fudge]
//SPDX-FileCopyrightText: © 2024 Robert Fudge <rnfudge@mun.ca>
//SPDX-License-Identifier: {Apache-2.0}

#include <cstdio>

#include <sys/types.h>
#include <sys/wait.h>

#include <stdlib.h>

#include "adtr2_base/AUVMonitor.hpp"

using namespace std::chrono_literals;
using namespace adtr2::monitor;

AUVMonitor::AUVMonitor(const rclcpp::NodeOptions & options) : Node("auv_monitor", options), count_(0) {
    //Set up timer for publishing statuses and timer for checking if nodes are active
    status_publishing_timer = this->create_wall_timer(250ms, std::bind(&AUVMonitor::__status_publishing_callback, this));
    node_checking_timer = this->create_wall_timer(250ms, std::bind(&AUVMonitor::__node_checking_callback, this));

    //Initialize process ids to 0
    daq_process = 0;
    system_process = 0;

    //Initialize component statuses to 0
    system_launcher_status = 0;
    zed_status = 0;
    vslam_status = 0;
    od_status = 0;
    
    daq_status = 0;
    system_status = 0;

    //Create services for toggling components
    daq_service = create_service<std_srvs::srv::Trigger>(package_prefix + "toggle_daq",
        std::bind(&AUVMonitor::toggle_daq, this, std::placeholders::_1, std::placeholders::_2));

    system_service = create_service<std_srvs::srv::Trigger>(package_prefix + "toggle_system",
        std::bind(&AUVMonitor::toggle_system, this, std::placeholders::_1, std::placeholders::_2));

    //Create publishers for component statuses
    daq_stat_publisher = create_publisher<example_interfaces::msg::UInt8>(package_prefix + "status/daq", max_messages);
    system_stat_publisher = create_publisher<example_interfaces::msg::UInt32>(package_prefix + "status/system", max_messages);

    //Create uint8_t messages for publishing status
    daq_status_msg = example_interfaces::msg::UInt8();
    system_status_msg = example_interfaces::msg::UInt32();

    //Initialize message data
    daq_status_msg.data = daq_status;
    system_status_msg.data = system_status;
}

AUVMonitor::~AUVMonitor() {
    //Not Implemented
}

void AUVMonitor::toggle_daq(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Starting data acquisition...");

    //If devices are not initialized
    if (!system_on) {
        RCLCPP_WARN(this->get_logger(), "System is not started. Cannot start data acquisition.");
        response->success = false;
        response->message = "Data acquisition not started, system is inactive.";
        return;
    }

    //If devices are initialized
    else {
        //Turn recording on and notify the user
        if (!daq_on) {
            pid_t n = fork();

            if (n == 0) {
                //Use execl to run ros2 launch on data acquisition launch script - in child process
                //NULL acts as sentinel
                int launch_result = execl("/bin/bash", "bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && source /home/${USERNAME}/ros_ws/install/setup.bash | ros2 launch adtr2_bringup daq.launch.py", NULL);

                if (launch_result != -1) {
                    RCLCPP_INFO(this->get_logger(), "Data acquisition script started...");
                }
                    
                else {
                    RCLCPP_ERROR(this->get_logger(), "Could not start data acquisition script.");
                    response->success = false;
                    response->message = "Data acquisition not started, failed to execute launch script.";
                    daq_on = false;
                    exit(1);
                }
            }

            //Store pid, format response, update internal member and return
            else {
                daq_process = n;
                response->success = true;
                response->message = "Data acquisition script started.";
                daq_on = true;
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
                response->message = "Data acquisition stopped successfully.";
                daq_on = false;
                return;
            }

            else {
                RCLCPP_WARN(this->get_logger(), "Error: Data acquisition process couldn't be killed, is it running?");
                response->success = false;
                response->message = "Data acquisition not stopped, failed to kill process.";
                daq_on = true;
                exit(1);
            }
        }
    }
}

void AUVMonitor::toggle_system(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    if (!system_on) {
        RCLCPP_INFO(this->get_logger(), "Starting system launch script...");

        pid_t n = fork();

        if (n == 0) {
            //Use execl to run ros2 launch on nv_vslam launch script - in child process
            int launch_result = execl("/bin/bash", "bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && source /home/${USERNAME}/ros_ws/install/setup.bash | ros2 launch adtr2_bringup auv.system.launch.py", NULL);

            if (launch_result != -1) {
                RCLCPP_INFO(this->get_logger(), "System launch script started successfully");
            }

            else {
                RCLCPP_ERROR(this->get_logger(), "Could not start System launch script.");
                response->success = false;
                response->message = "System not started, failed to execute nv_vslam script.";
                system_on = false;
                exit(1);
            }
        }

        else {
            system_process = n;
            response->success = true;
            response->message = "System started.";
            system_on = true;
            return;
        }
    }

    //If Visual SLAM is initialized
    else {
        //Stop Visual SLAM
        int stop_result = kill(system_process, SIGINT);

        if (stop_result != -1) {
            waitpid(system_process, nullptr, 0);
            RCLCPP_INFO(this->get_logger(), "System stopped successfully!");
            response->success = true;
            response->message = "System stopped successfully.";
            system_on = false;
            return;
        }

        else {
            RCLCPP_WARN(this->get_logger(), "Error: System process couldn't be killed, is it running?");
            response->success = false;
            response->message = "System not stopped, failed to kill process.";
            system_on = true;
            exit(1);
        }
    }
}
