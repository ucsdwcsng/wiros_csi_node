#pragma once
#include <stdio.h>
#include <vector>
#include "nex_data.hpp"
#ifdef WIROS_ROS2
#define ROS_INFO(...) printf(__VA_ARGS__)
#define ROS_ERROR(...) printf(__VA_ARGS__)
#define ROS_FATAL(...) printf(__VA_ARGS__)
#define ROS_WARN(...) printf(__VA_ARGS__)
#else
#include "ros/ros.h"
#endif
void ros_setup(void);
void publish_csi(const std::vector<csi_instance> &channel, std::string rx_ip);
