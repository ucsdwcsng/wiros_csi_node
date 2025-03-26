#pragma once
#include <stdio.h>
#include <vector>
#include "nex_data.hpp"
#ifdef WIROS_ROS2
#include "ros2_interface.hpp"
#define ROS_INFO(...) RCLCPP_INFO(g_node->get_logger(), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(g_node->get_logger(), __VA_ARGS__)
#define ROS_FATAL(...) RCLCPP_FATAL(g_node->get_logger(), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(g_node->get_logger(),__VA_ARGS__)
#else
#include "ros/ros.h"
#endif
void ros_setup(void);
void publish_csi(const std::vector<csi_instance> &channel, std::string rx_ip);
