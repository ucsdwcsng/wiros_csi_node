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
      publisher_ = this->create_publisher<rf_msgs::msg::Wifi>("csi", 10);
    }

  void run(void){
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

  void publish_csi(const std::vector<csi_instance> &channel, std::string rx_ip){
    auto msgout = rf_msgs::msg::Wifi();
    if (channel.size() == 0){
      return;
    }
    
    size_t rx_stride = channel[0].n_sub;
    size_t rx2 = rx_stride / 2;
    size_t tx_stride = rx_stride*4;
    size_t num_floats = tx_stride*4;
    msgout.header.stamp = this->now();
    msgout.ap_id = 0;
    msgout.txmac = std::vector<unsigned char>(channel[0].source_mac, channel[0].source_mac + 6);
    msgout.chan = channel[0].channel;
    msgout.n_sub = channel[0].n_sub;
    msgout.seq_num = channel[0].seq;
    msgout.fc = channel[0].fc;
    msgout.n_rows = 4;
    msgout.n_cols = 4;
    msgout.bw = channel[0].bw;
    msgout.mcs = 0;
    msgout.rssi = (int32_t)(channel[0].rssi);
    printf("%s:RSSI%d/seq%d/fc%.2hhx/chan%d\n",mac_to_string(channel[0].source_mac).c_str(), msgout.rssi, msgout.seq_num, channel[0].fc, msgout.chan);

    msgout.csi_real = std::vector<double>(num_floats);
    msgout.csi_imag = std::vector<double>(num_floats);
    
    msgout.rx_id = rx_ip;
    msgout.msg_id = 0;
    for(auto c = channel.begin(); c != channel.end(); ++c){
      size_t csi_idx = rx_stride*c->rx + tx_stride*c->tx;
      //fft-shift the data
      for(int bin = 0; bin < rx2; ++bin){
	msgout.csi_real[csi_idx + bin + rx2] = (c->csi_r[bin]);
	msgout.csi_imag[csi_idx + bin + rx2] = (c->csi_i[bin]);
	msgout.csi_real[csi_idx + bin] = (c->csi_r[bin + rx2]);
	msgout.csi_imag[csi_idx + bin] = (c->csi_i[bin + rx2]);
      }
    }
    
    publisher_->publish(msgout);
  }
  
private:
  rclcpp::Publisher<rf_msgs::msg::Wifi>::SharedPtr publisher_;
};

std::unique_ptr<csi_node> g_node;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  g_node = std::make_unique<csi_node>();
  g_node->run();
  g_node.release();
  rclcpp::shutdown();
  return 0;
}

void publish_csi(const std::vector<csi_instance> &channel, std::string rx_ip){
  g_node->publish_csi(channel, rx_ip);
}
