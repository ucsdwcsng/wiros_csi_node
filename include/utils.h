#include <vector>
#include <regex>

const std::regex addr_regex("(..|\\*):(..|\\*):(..|\\*):(..|\\*):(..|\\*):(..|\\*)");
const std::regex ip_regex("([0-9]{1,3})\\.([0-9]{1,3})\\.([0-9]{1,3})\\.([0-9]{1,3}|\\*)");
const unsigned char mac_zero[6] = {0,0,0,0,0,0};

class mac_filter{
 public:
  size_t len;
  uint8_t mac[6];
  mac_filter(){
	len=0;
	for(int i = 0; i < 6; ++i){
	  mac[i] = 0;
	}
  }
  mac_filter(int i_len, uint8_t i_mac[6]){
	len = i_len;
	for(int i = 0; i < 6; ++i){
	  mac[i] = i < len ? i_mac[i] : 0;
	}
  }
};

//removes non-alphanumeric characters from the command output
inline bool sanitize_string(char c){
  return !(c == ' ' || (c>= 32 && c <= 176));
}

//human readable mac address
std::string hr_mac(const unsigned char* source_mac)
{
  char source_mac_str[19];
  sprintf(source_mac_str, "%.2hhx:%.2hhx:%.2hhx:%.2hhx:%.2hhx:%.2hhx", source_mac[0], source_mac[1], source_mac[2], source_mac[3], source_mac[4], source_mac[5]);
  return std::string(source_mac_str);
}

//human readable ip address
std::string hr_ip(unsigned char* source_ip)
{
  char source_ip_str[19];
  sprintf(source_ip_str, "%d.%d.%d.%d", source_ip[0], source_ip[1], source_ip[2], source_ip[3]);
  return std::string(source_ip_str);
}

bool mac_cmp(const unsigned char* a, const mac_filter b){
  for(int i = 0; i < b.len; ++i){
	if(a[i] != b.mac[i]) return false;
  }
  return true;
}


//human readable mac address
std::string hr_mac_filt(mac_filter filter)
{
  char print_blk[6][4];
  for(int i = 0; i < 6; ++i){
    if(i < filter.len)
      sprintf(print_blk[i],"%.2hhx",filter.mac[i]);
    else
      sprintf(print_blk[i],"*");
  }
  char source_mac_str[64];
  sprintf(source_mac_str, "%s:%s:%s:%s:%s:%s", print_blk[0], print_blk[1], print_blk[2], print_blk[3], print_blk[4], print_blk[5]);
  return std::string(source_mac_str);
}


mac_filter mac_filter_str(std::string in_str){
  if(in_str == "") return mac_filter();
  
  std::smatch mac_match;
  mac_filter ret;
  if(std::regex_search(in_str,mac_match,addr_ex)){
	int i;
	for(i=1 ; i < mac_match.size(); ++i){
      if(mac_match[i].str() != "*"){
        ret.mac[i - 1] = (uint8_t)std::strtol(mac_match[i].str().c_str(), NULL, 16);
      }
      else break;
	}
	ret.len = i - 1;
  }
  else{
  	ROS_FATAL("Invalid MAC address for filter: %s, should only contain 1-byte hex digits and *", in_str.c_str());
  	exit(EXIT_FAILURE);
  }
  
  return ret;
}
