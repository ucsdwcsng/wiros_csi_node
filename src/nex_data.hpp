#pragma once

#include <stdint.h>
#include <cstdlib>

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
  std::vector<double> csi_r;
  std::vector<double> csi_i;
  uint16_t seq;
  uint8_t fc;
};


struct csi_udp_frame {
    uint8_t kk1;//magic number
    uint8_t id;
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

//human readable mac address
std::string mac_to_string(unsigned char const* source_mac);
