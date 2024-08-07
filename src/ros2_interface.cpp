#include "rf_msgs/msg/wifi.hpp"

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nex_conf.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class csi_node : public rclcpp::Node
{
  public:
    csi_node()
    : Node("minimal_publisher")
    {
      publisher_ = this->create_publisher<rf_msgs::msg::Wifi>("/csi", 10);
      //determine our hostname
      std::string hostname = sh_exec_block("hostname");

      nex_config_t param = {
      use_tcp_forward:false,
      lock_topic:"/",
      csi_config: {
        channel: 36,
        bw: 80,
        beacon_rate:20,
        beacon_mac_4:(uint8_t)hostname[0],
        beacon_mac_5:(uint8_t)hostname[1],
        beacon_mac_6:0,
        beacon_tx_streams:4,
        dev_ip:"192.168.44.4",
        dev_password:"robot123!",
        dev_hostname:"wcsng",
        rx_mac_filter: mac_filter_t("*:*:*:*:*:*")
      }
      };
      wiros_main(param);
    }

  void publish_csi(const std::vector<csi_instance> &channel){
    rf_msgs::msg::Wifi msg;
    if (channel.size() == 0){
      return;
    }
    msg.ap_id = 0;
    msg.txmac = std::vector<uint8_t>(&channel[0].source_mac[0], &channel[0].source_mac[6]);
    publisher_->publish(msg);
    printf("PUB\n");
  }
  
  private:
  rclcpp::Publisher<rf_msgs::msg::Wifi>::SharedPtr publisher_;
};

std::unique_ptr<csi_node> g_node;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  g_node = std::make_unique<csi_node>();
  g_node.release();
  rclcpp::shutdown();
  return 0;
}

void publish_csi(const std::vector<csi_instance> &channel){
  g_node->publish_csi(channel);
}
