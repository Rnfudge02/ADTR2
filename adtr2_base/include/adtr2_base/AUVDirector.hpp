//Copyright [2024] [Robert Fudge]
//SPDX-FileCopyrightText: © 2024 Robert Fudge <rnfudge@mun.ca>
//SPDX-License-Identifier: Apache-2.0
#pragma once

//Standard library includes
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <cstdlib>
#include <unistd.h>

#include <sys/types.h>
#include <sys/wait.h>

//ROS2 includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "adtr2_base/ADTR2Module.hpp"

#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/bounding_box2_d.hpp"

namespace adtr2 {
    namespace director {
        //! AUVDirector - Controls the device based on environmental information.
        /*!
            Responsible for interacting with the object detection and stereo depth perception ros2 packages,
            and will use the data collected to determine the movement of the AUV.
        !*/
        class AUVDirector : public ADTR2Module {
        public:
            //! Creates an AUVDirector Node.
            /*!
                Initializes class members to a known state and sets up required ROS2 services and publisher topics
                for detection of objects.

                @param options Options structure used to pass launch arguments to class.
                @return Instance of AUVDirector.
            !*/
            AUVDirector(const rclcpp::NodeOptions& options);

        protected:

        private:
            //! Callback to check detections.
            /*!
                Contains logic to check the detections since last call.
                
                @return None.
            !*/
            void detection_timer_callback() {

                return;
            }

            //! Callback to communicate intended direction to the Controller module.
            /*!


                @return None.
            !*/
            void direction_timer_callback() {
        
                return;
            }

            //! Callback to publish node heartbeat
            /*!
                
                @return None.
            !*/
            void status_timer_callback() {
                internal_status_msg.data = internal_status;
                internal_status_publisher->publish(internal_status_msg);
            }

            //! Callback for handling detected images
            /*!
                

                @param msg Message being handled in the callback
                @return Instance of AUVMonitor.
            !*/
            void detection_img_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "Received a Detection2DArray message with %zu detections", msg->detections.size());

                //Get all detections
                //vision_msgs::msg::Detection2DArray det_array = *msg;

                //det_array.insert(det_array.detections.end(), new_detections.detections.begin(), new_detections.detections.end());

 
            }

            //! Analyses visual SLAM messages via callback
            /*!
                @return Instance of AUVMonitor.
            !*/
            void vslam_img_callback() {

            }

            //
            uint8_t internal_status;
            std_msgs::msg::UInt8 internal_status_msg;

            //
            rclcpp::TimerBase::SharedPtr detection_timer;
            rclcpp::TimerBase::SharedPtr direction_timer;
            rclcpp::TimerBase::SharedPtr status_timer;

            //
            rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr internal_status_publisher;

            uint8_t quadrant_arr;

            //
            rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_subscriber;

            std_msgs::msg::UInt8 quadrant_arr_msg;
            rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr quadrant_to_nav_publisher;

            //Assuming is used to allow only one instance of the class to exist
            size_t count_;
        };
    }
}

//
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(adtr2::director::AUVDirector);