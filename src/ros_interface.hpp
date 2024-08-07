#pragma once
#include <stdio.h>
#include <vector>
#include "nex_data.hpp"
#define ROS_INFO(...) printf(__VA_ARGS__)
#define ROS_ERROR(...) printf(__VA_ARGS__)
#define ROS_FATAL(...) printf(__VA_ARGS__)
#define ROS_WARN(...) printf(__VA_ARGS__)

void ros_setup(void);
void publish_csi(const std::vector<csi_instance> &channel);
