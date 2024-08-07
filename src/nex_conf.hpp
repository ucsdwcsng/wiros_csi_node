#pragma once
#include <string>
#include <cstring>
#include <regex>
#include <unistd.h>

#include "shell_utils.hpp"
#include "ros_interface.hpp"

const std::regex ip_ex("([0-9]{1,3})\\.([0-9]{1,3})\\.([0-9]{1,3})\\.([0-9]{1,3}|\\*)");
const std::regex addr_ex("(..|\\*):(..|\\*):(..|\\*):(..|\\*):(..|\\*):(..|\\*)");

class mac_filter_t{
 public:
  size_t len;
  uint8_t mac[6];
  mac_filter_t(){
    len=0;
    memset(mac,0,6);
  }
  mac_filter_t(int i_len, uint8_t i_mac[6]){
    len = i_len;
    for(int i = 0; i < 6; ++i){
      mac[i] = i < len ? i_mac[i] : 0;
    }
  }
  mac_filter_t(std::string filt_str){
    if(filt_str == ""){
      len=0;
      memset(mac,0,6);
      return;
    }
    std::smatch mac_match;
    if(std::regex_search(filt_str,mac_match,addr_ex)){
      int i;
      for(i=1 ; i < mac_match.size(); ++i){
	if(mac_match[i].str() != "*"){
	  mac[i - 1] = (uint8_t)std::strtol(mac_match[i].str().c_str(), NULL, 16);
	}
	else break;
      }
      len = i - 1;
    }
    else{
      ROS_FATAL("Invalid MAC address for filter: %s, should only contain 1-byte hex digits and *", filt_str.c_str());
      exit(EXIT_FAILURE);
    }
  }
  bool matches(uint8_t m[6]) const{
    for(int i = 0; i < len; ++i){
      if(m[i] != mac[i]) return false;
    }
    return true;
  }

};


typedef struct csi_config{
  int channel;
  int bw;
  double beacon_rate;
  uint8_t beacon_mac_4;
  uint8_t beacon_mac_5;
  uint8_t beacon_mac_6;
  int beacon_tx_streams;
  std::string dev_ip;
  std::string dev_password;
  std::string dev_hostname;
  mac_filter_t rx_mac_filter;
} csi_config_t;


typedef struct nex_config{
  bool use_tcp_forward;
  std::string lock_topic;
  csi_config_t csi_config;
} nex_config_t;



std::string apply_csi_config(csi_config_t conf){
  std::string iface;
  char configcmd[384];
  std::string ret;
  mac_filter_t filter(conf.rx_mac_filter);
  bool configured = false;
  std::string output;
  if(conf.channel >= 32){
      iface = "eth6";
      if(conf.beacon_tx_streams > 4) conf.beacon_tx_streams = 4;
  }
  else{
      iface = "eth5";
      if(conf.beacon_tx_streams > 3) conf.beacon_tx_streams = 3;
  }
  
  if(filter.len > 1){
	sprintf(configcmd, "sshpass -p %s ssh -o strictHostKeyChecking=no %s@%s /jffs/csi/setup.sh %d %d 4 %.2hhx:%.2hhx:00:00:00:00 2>&1",
            conf.dev_password.c_str(), conf.dev_hostname.c_str(), conf.dev_ip.c_str(), conf.channel, conf.bw, conf.rx_mac_filter.mac[0],conf.rx_mac_filter.mac[1]);
  }
  else{
	sprintf(configcmd, "sshpass -p %s ssh -o strictHostKeyChecking=no %s@%s /jffs/csi/setup.sh %d %d 4 2>&1", conf.dev_password.c_str(), conf.dev_hostname.c_str(), conf.dev_ip.c_str(), conf.channel, conf.bw);
  }

  ret += "RUN: " + std::string(configcmd);
  
  do{
    output = sh_exec_block(configcmd);
    ROS_INFO("\n***\nSetup Output:\n\n%s\n***", output.c_str());
    if(output.find("Permission denied") != std::string::npos){
      ROS_ERROR("A device was found at %s, but it refused SSH access.\nPlease check the 'asus_pwd' param and ensure it is set to the device's password.\nCurrent passsword: %s\nThis may also be caused by setup scripts not having the correct permissions set.",conf.dev_ip.c_str(),conf.dev_password.c_str());
      exit(EXIT_FAILURE);
    } else if(output.find("Connection refused") != std::string::npos){
      ROS_INFO("Waiting 5 seconds and retrying...");
      sleep(5);
    } else {
      configured = true;
    }
  } while(!configured);
  
  ret += "RESULT: " + output;
  return ret;
}
