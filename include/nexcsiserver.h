//
// Created by wcsng-ros on 3/28/22.
//

#ifndef NEXMON_CSI_ROS_NEXCSISERVER_H
#define NEXMON_CSI_ROS_NEXCSISERVER_H

#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <ros/ros.h>
#include <sstream>
#include <queue>
#include <vector>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <ctime>
#include <regex>

//https://github.com/ucsdwcsng/rf_msgs.git
#include "rf_msgs/Wifi.h"
#include "shutils.h"
#include "nexmon_csi_ros/ConfigureCSI.h"
#include "rf_msgs/Station.h"
#include "rf_msgs/AccessPoints.h"

#define SA struct sockaddr

#define MAXLINE 1024
#define CSI_BUF_SIZE 4096
#define PORT 5500
#define PORT_TCP 50005
#define CSI_HDR_START_CMP "\x0a\x0a\x0a\x0a\xff\xff\xff\xff"
#define NO_HDR_IN_BUF -1
#define NEW_CSI_HDR 8
#define CSI_OFFSET 16

#define k_tof_unpack_sgn_mask (1<<31)
#define H_OFFSET 64
#define TIMESTAMP_BYTE_OFFSET 42

#define stop() signal(2)

const char* chan_arg = "-c";
const char* bw_arg = "-b";
const char* mac_arg = "-m";
const char* rx_arg = "-r";

const char* rx_host = "wcsng";


//111111 - exponent of 
const uint32_t e_mask = (1<<6)-1;
//111111111111
const uint32_t mantissa_mask = (1<<12)-1;
//1
const uint32_t sign_mask = 1;
//111111111111000000
const uint32_t r_mant_mask = (((1<<11) - 1) << 18);
//same for imag
const uint32_t i_mant_mask = (((1<<11) - 1) << 6);
//
const uint32_t r_sign_mask = (1<<29);
const uint32_t i_sign_mask = (1<<17);

const uint32_t count_mask = (1<<10);
const uint32_t mant_mask = (1<<10)-1;

const std::regex addr_ex("(..|\\*):(..|\\*):(..|\\*):(..|\\*):(..|\\*):(..|\\*)");
const std::regex ip_ex("([0-9]{1,3})\\.([0-9]{1,3})\\.([0-9]{1,3})\\.([0-9]{1,3}|\\*)");
const unsigned char mac_zero[6] = {0,0,0,0,0,0};

//holds the remote client's ssh process so we can shut it down properly
FILE* cli_fp = NULL;
//same for the TX process
FILE* tx_fp = NULL;

class csi_instance
{
public:
    uint8_t source_mac[6];
    uint16_t seq_num;
    int8_t rssi;
    uint8_t tx;
    uint8_t rx;
    uint8_t channel;
    uint8_t bw;
    size_t n_sub;
    double* csi_r;
    double* csi_i;
    uint16_t seq;
    uint8_t fc;
    ~csi_instance(){
      if(!csi_r)
        delete csi_r;
      if(!csi_i)
        delete csi_i;
    }
};


struct csi_udp_frame {
    uint16_t kk1;//magic number
    int8_t rssi;//rssi
    uint8_t fc; //frame control
    uint8_t src_mac[6];//source mac
    uint16_t seqCnt;//frame sequence number
    uint16_t csiconf;// core + spatial stream
    uint16_t chanspec;//chanspec
    uint16_t chip;//chip version

    //then follows the csi bytes, for 80MHZ:
    //uint32_t csi_values[512];
};

//main functions

//sets up rosparams
void setup_params(ros::NodeHandle& nh);

//parses csi from bytes
void parse_csi(unsigned char* data, size_t nbytes);

//create ros message
void publish_csi(std::vector<csi_instance> &channel_current);

//close the active processes on asus
void handle_shutdown(int sig);

//update CSI filter settings on asus
std::string reconfigure();

void setup_tcpdump(std::string hostIP);

bool set_chanspec(int s_chan, int s_bw);

bool set_mac_filter(std::vector<int> filt);
bool set_mac_filter(const uint8_t* mac);

bool config_csi_callback(nexmon_csi_ros::ConfigureCSI::Request &req, nexmon_csi_ros::ConfigureCSI::Response &resp);

void ap_info_callback(const rf_msgs::AccessPoints::ConstPtr& msg);

//helper functions


//removes non-alphanumeric characters from the command output
inline bool sanitize_string(char c){
  return !(c == ' ' || (c>= 32 && c <= 176));
}


//search for new packets in the data stream
inline bool is_csi_hdr(unsigned char* data){
  for(int i = 0; i < NEW_CSI_HDR/2; ++i){
    if (*(data + i) != (unsigned char)'\x0a'){
      return false;
    }
  }
  for(int i = 4; i < NEW_CSI_HDR; ++i){
    if (*(data + i) != (unsigned char)'\xff'){
      return false;
    }
  }
  return true;
};

//locate the start of a CSI udp packet in the received tcpdump data
size_t find_csi_hdr(unsigned char* csidata){
  size_t end = CSI_BUF_SIZE - NEW_CSI_HDR;
  for(size_t i = NEW_CSI_HDR; i < end; ++i){
    if(is_csi_hdr(csidata + i))
      return i;
  }
  return NO_HDR_IN_BUF;
}

//human readable mac address
std::string hr_mac(const unsigned char* source_mac)
{
  char source_mac_str[19];
  sprintf(source_mac_str, "%.2hhx:%.2hhx:%.2hhx:%.2hhx:%.2hhx:%.2hhx", source_mac[0], source_mac[1], source_mac[2], source_mac[3], source_mac[4], source_mac[5]);
  return std::string(source_mac_str);
}

//human readable mac address
std::string hr_mac_filt(std::vector<int> filter)
{
  char print_blk[6][4];
  for(int i = 0; i < 6; ++i){
    if(i < filter.size())
      sprintf(print_blk[i],"%.2hhx",filter[i]);
    else
      sprintf(print_blk[i],"*");
  }
  char source_mac_str[64];
  sprintf(source_mac_str, "%s:%s:%s:%s:%s:%s", print_blk[0], print_blk[1], print_blk[2], print_blk[3], print_blk[4], print_blk[5]);
  return std::string(source_mac_str);
}

std::string u32_bits(uint32_t x){
  std::stringstream ss;
  for(int i = 31; i >= 0; --i){
    ss << (x>>i & 1) ? "1" : "0";
  }
  return ss.str();
}

std::string u32_bits_db(uint32_t x){
  std::stringstream ss;
  for(int i = 31; i >= 0; --i){
    if(i == 29 || i == 28 || i == 17 || i == 16 || i == 5)
      ss << " ";
    ss << (x>>i & 1) ? "1" : "0";
  }
  return ss.str();
}


std::string u64_bits(uint64_t x){
  std::stringstream ss;
  for(int i = 63; i >= 0; --i){
    if(i == 62 || i == 51)
      ss << " ";
    ss << (x>>i & 1) ? "1" : "0";
  }
  return ss.str();
}
void dbg_csi_raw(uint32_t c){
  ROS_INFO("raw bits:\t%s", u32_bits(c).c_str());
  ROS_INFO("e:\t%s",u32_bits_db(c&e_mask).c_str());
  ROS_INFO("sr:\t%s",u32_bits_db(c&r_sign_mask).c_str());
  ROS_INFO("mr:\t%s",u32_bits_db(c&r_mant_mask).c_str());
  ROS_INFO("si:\t%s",u32_bits_db(c&i_sign_mask).c_str());
  ROS_INFO("mi:\t%s",u32_bits_db(c&i_mant_mask).c_str());
}

//human readable ip address
std::string hr_ip(unsigned char* source_ip)
{
  char source_ip_str[19];
  sprintf(source_ip_str, "%d.%d.%d.%d", source_ip[0], source_ip[1], source_ip[2], source_ip[3]);
  return std::string(source_ip_str);
}

//two bytes to uint16_t
uint16_t char2u16(char u, char l)
{
  return (((uint16_t)u) << 8) | (0x00ff & l);
};

bool mac_cmp(const unsigned char* a, const unsigned char* b){
  if(a[0] == b[0] && a[1] == b[1] && a[2] == b[2] && a[3] == b[3] && a[4] == b[4] && a[5] == b[5])
    return true;
  return false;
}

std::vector<int> mac_filter_strtovec(std::string in_str){
  if(in_str == "") return std::vector<int>();
  
  std::smatch mac_match;
  std::vector<int> ret;
  if(std::regex_search(in_str,mac_match,addr_ex)){
	for(int i = 1; i < mac_match.size(); ++i){
	  if(mac_match[i].str() != "*"){
		ret.push_back(std::strtol(mac_match[i].str().c_str(), NULL, 16));
	  }
	}
  }

  else{
	ROS_FATAL("Invalid MAC address for filter: %s, should only contain 1-byte hex digits and *", in_str.c_str());
	exit(EXIT_FAILURE);
  }

  return ret;
}


#endif //NEXMON_CSI_ROS_NEXCSISERVER_H
