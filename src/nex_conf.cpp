#include "nex_conf.hpp"
#include "ros_interface.hpp"

// Define regex expressions
const std::regex ip_ex("([0-9]{1,3})\\.([0-9]{1,3})\\.([0-9]{1,3})\\.([0-9]{1,3}|\\*)");
const std::regex addr_ex("(..|\\*):(..|\\*):(..|\\*):(..|\\*):(..|\\*):(..|\\*)");

// Default constructor
mac_filter_t::mac_filter_t() {
    len = 0;
    memset(mac, 0, 6);
}

// Constructor with specified length and MAC array
mac_filter_t::mac_filter_t(int i_len, uint8_t i_mac[6]) {
    len = i_len;
    for (int i = 0; i < 6; ++i) {
        mac[i] = (i < len) ? i_mac[i] : 0;
    }
}

// Constructor with MAC filter string
mac_filter_t::mac_filter_t(const std::string& filt_str) {
    if (filt_str.empty()) {
        len = 0;
        memset(mac, 0, 6);
        return;
    }

    std::smatch mac_match;
    if (std::regex_search(filt_str, mac_match, addr_ex)) {
        int i;
        for (i = 1; i < mac_match.size(); ++i) {
            if (mac_match[i].str() != "*") {
                mac[i - 1] = static_cast<uint8_t>(std::strtol(mac_match[i].str().c_str(), nullptr, 16));
            } else {
                break;
            }
        }
        len = i - 1;
    } else {
      ROS_ERROR("Invalid MAC filter string format!\n");
      exit(EXIT_FAILURE);
    }
}

// Check if a given MAC address matches the filter
bool mac_filter_t::matches(const uint8_t m[6]) const {
    for (int i = 0; i < len; ++i) {
        if (m[i] != mac[i]) return false;
    }
    return true;
}
