#include "rf_msgs/Wifi.h"
#include "ros_interface.hpp"
#include "ros/ros.h"
#include "nex_conf.hpp"

//publisher
ros::Publisher pub_csi;

int main(int argc, char* argv[]){

  //setup ros
  ros::init(argc, argv, "nexcsi", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh("~");

  ros::AsyncSpinner spinner(0);
  spinner.start();

  nex_config_t param;
  double tmp_chan, tmp_bw, tmp_beacon;
  nh.param<double>("channel", tmp_chan, 157.0);
  param.csi_config.channel = tmp_chan;
  nh.param<double>("bw", tmp_bw, 80.0);
  param.csi_config.bw = tmp_bw;
  nh.param<double>("beacon_rate", tmp_beacon, 200.0);
  param.csi_config.beacon_rate = tmp_beacon;
  nh.param<int>("beacon_tx_nss", param.csi_config.beacon_tx_streams, 4);
  nh.param<bool>("tcp_forward", param.use_tcp_forward, false);
  nh.param<std::string>("asus_ip", param.csi_config.dev_ip, "");
  nh.param<std::string>("asus_pwd", param.csi_config.dev_password, "password");
  nh.param<std::string>("asus_host", param.csi_config.dev_hostname, "HOST");
  //nh.param<std::string>("lock_topic", lock_topic, "");
  //MAC filter param
  std::string mac_filter_temp;
  nh.param<std::string>("mac_filter", mac_filter_temp, std::string(""));
  param.csi_config.rx_mac_filter = mac_filter_t(mac_filter_temp);

  //determine our hostname
  std::string hostname = sh_exec_block("hostname");
  param.csi_config.beacon_mac_4 = (uint8_t)hostname[0];
  param.csi_config.beacon_mac_5 = (uint8_t)hostname[1];

  pub_csi = nh.advertise<rf_msgs::Wifi>("csi",10);
  ROS_INFO("Publishing: %s", pub_csi.getTopic().c_str());

  wiros_main(param);
}

void publish_csi(std::vector<csi_instance> const &channel_current, std::string rx_ip){
  //4x4 matrices, with n_sub elements each, w/ interleaved 4 byte real + imag parts
  csi_instance csi_0 = channel_current.at(0);
  size_t rx_stride = csi_0.n_sub;
  size_t rx2 = rx_stride / 2;
  size_t tx_stride = rx_stride*4;
  size_t num_floats = tx_stride*4;
  rf_msgs::Wifi msgout;
  msgout.header.stamp = ros::Time::now();
  msgout.ap_id = 0;
  msgout.txmac = std::vector<unsigned char>(csi_0.source_mac, csi_0.source_mac + 6);
  msgout.chan = csi_0.channel;
  msgout.n_sub = csi_0.n_sub;
  msgout.seq_num = csi_0.seq;
  msgout.fc = csi_0.fc;
  msgout.n_rows = 4;
  msgout.n_cols = 4;
  msgout.bw = csi_0.bw;
  msgout.mcs = 0;
  msgout.rssi = (int32_t)(csi_0.rssi);
  ROS_INFO("%s:RSSI%d/seq%d/fc%.2hhx/chan%d/rx%s",mac_to_string(csi_0.source_mac).c_str(), msgout.rssi, msgout.seq_num, csi_0.fc, msgout.chan, rx_ip.c_str());
  msgout.csi_real = std::vector<double>(num_floats);
  msgout.csi_imag = std::vector<double>(num_floats);
  msgout.rx_id = rx_ip;
  msgout.msg_id = 0;
  for(auto c = channel_current.begin(); c != channel_current.end(); ++c){
	size_t csi_idx = rx_stride*c->rx + tx_stride*c->tx;
	//fft-shift the data
	for(int bin = 0; bin < rx2; ++bin){
	  msgout.csi_real[csi_idx + bin + rx2] = (c->csi_r[bin]);
	  msgout.csi_imag[csi_idx + bin + rx2] = (c->csi_i[bin]);
	  msgout.csi_real[csi_idx + bin] = (c->csi_r[bin + rx2]);
	  msgout.csi_imag[csi_idx + bin] = (c->csi_i[bin + rx2]);
	}
  }
  pub_csi.publish(msgout);
}
