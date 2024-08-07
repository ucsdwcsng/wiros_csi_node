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


void wiros_main(nex_config_t &param);
