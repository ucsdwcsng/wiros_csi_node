#pragma once
#include "rf_msgs/msg/wifi.hpp"

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "nex_conf.hpp"


using namespace std::chrono_literals;

class csi_node : public rclcpp::Node{
public:
  csi_node();
  void run(void);
  void publish_csi(const std::vector<csi_instance> &channel, std::string rx_ip);
private:
  rclcpp::Publisher<rf_msgs::msg::Wifi>::SharedPtr publisher_;
};

extern std::unique_ptr<csi_node> g_node;
