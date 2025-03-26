#pragma once
#include "ros_interface.hpp"
#include "shell_utils.hpp"

#include <string>
#include <cstring>
#include <regex>
#include <unistd.h>
#include <vector>

// Regular expressions for matching IP and MAC addresses
extern const std::regex ip_ex;
extern const std::regex addr_ex;

// Class to represent MAC address filtering
class mac_filter_t {
public:
    size_t len;
    uint8_t mac[6];

    mac_filter_t();
    mac_filter_t(int i_len, uint8_t i_mac[6]);
    mac_filter_t(const std::string& filt_str);

    bool matches(const uint8_t m[6]) const;
};

// Structure to hold CSI configuration
struct csi_config_t {
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
};

// Structure to hold NEX configuration
struct nex_config_t {
    bool use_tcp_forward;
    std::string lock_topic;
    csi_config_t csi_config;
};

// Function prototype for main processing function
void wiros_main(nex_config_t &param);
