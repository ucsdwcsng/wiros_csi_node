#include <cstring>
#include <regex>
#include <stdint.h>
#include <array>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <signal.h>

#include "nex_conf.hpp"
#include "shell_utils.hpp"
#include "ros_interface.hpp"
#include "nex_data.hpp"

#define CSI_BUF_SIZE 4096
#define PORT 5500
#define PORT_TCP 50005

volatile bool g_ok = true;
//tracks our IP
std::string g_host_ip = "";
//The ASUS sends multiple packets for a given CSI measurement -
//if we have a 4x4 channel, 16 CSI packets will be sent.
std::vector<csi_instance> g_mimo_channel;
//used in figuring out if a CSI packet corresponds to the previous channel
//or a new one.
uint16_t g_last_seq;

//forward decl of local functions
void handle_shutdown(int sig);
//nex_config_t setup_params(ros::NodeHandle& nh);
void contact_device(nex_config_t& params);

std::array<uint8_t, CSI_BUF_SIZE> g_csi_buf;

void handle_shutdown(int sig){
  g_ok  = false;
}

void contact_device(nex_config_t& params){
  //automatically find connected asus router
  std::smatch ip_match;
  char subnet[20];
  bool scan=false;
  if(std::regex_search(params.csi_config.dev_ip,ip_match,ip_ex)){
	sprintf(subnet, "%s.%s.%s.", ip_match[1].str().c_str(), ip_match[2].str().c_str(), ip_match[3].str().c_str());
	if(ip_match[4]=="*"){  
	  scan=true;
	}
  }
  else{
	ROS_FATAL("Invalid target IP, needs to be xxx.xxx.xxx.xxx or xxx.xxx.xxx.*");
  }

  std::stringstream IPs(sh_exec_block("hostname -I"));
  std::string IP;
  bool iface_up = false;
  while(getline(IPs, IP, ' ')){
	if (IP.rfind(subnet, 0) == 0) {
	  g_host_ip = IP;
	  iface_up = true;

	  if(scan){
		char cmd[256];
		ROS_INFO("Scanning for ASUS routers...");
		sprintf(cmd, "nmap -sP %s0/24", subnet);
		ROS_INFO("%s", cmd);
		std::stringstream nmap(sh_exec_block(cmd));
		std::string target;
		while(getline(nmap, target, ' ')){
		  if (target.rfind(subnet, 0) == 0){
			std::string temp_ip = target.substr(0, target.find("\n"));
			if(temp_ip != g_host_ip){
              params.csi_config.dev_ip = temp_ip;
			}
		  }
		}
	  }
	}
  }
  char setupcmd[512];
  sprintf(setupcmd, "ping -c 3 -i 0.3 %s", params.csi_config.dev_ip.c_str());
  std::string ping_result = sh_exec_block(setupcmd);
  ROS_INFO("%s", ping_result.c_str());
  if(ping_result.find("Destination Host Unreachable",0) != std::string::npos || ping_result.find("100% packet loss",0) != std::string::npos){
	ROS_ERROR("The host at %s did not respond to a ping.", params.csi_config.dev_ip.c_str());
	ROS_ERROR("This is probably because the 'asus_ip' param is setup to the incorrect value.");
	ROS_ERROR("You can enable automatic ASUS detection by setting 'asus_ip' to \"\"");
	exit(1);
  }
  if(!iface_up){
	ROS_ERROR("The subnet does not appear to be active.");
	exit(1);
  }

}

char configcmd[512];
// void
void configure_device(nex_config_t& param){
  ROS_INFO("Configuring Receiver at %s...\n", param.csi_config.dev_ip.c_str());
  std::string iface;
  std::string cmd_output;
  //reset iface
  if(param.csi_config.channel >= 32){
      iface = "eth6";
	  param.csi_config.beacon_tx_streams = param.csi_config.beacon_tx_streams > 4 ? 4 : param.csi_config.beacon_tx_streams;
  }
  else{
      iface = "eth5";
	  param.csi_config.beacon_tx_streams = param.csi_config.beacon_tx_streams > 3 ? 3 : param.csi_config.beacon_tx_streams;
  }
  
  sprintf(configcmd, "sshpass -p %s ssh -o strictHostKeyChecking=no %s@%s killall send.sh",
            param.csi_config.dev_password.c_str(), param.csi_config.dev_hostname.c_str(), param.csi_config.dev_ip.c_str());
  printf("---RUN---\n %s\n",configcmd);
  printf("%s\n",sh_exec_block(configcmd).c_str());
  

    
  if(param.csi_config.rx_mac_filter.len > 1){
    
	sprintf(configcmd, "sshpass -p %s ssh -o strictHostKeyChecking=no %s@%s /jffs/csi/setup.sh %d %d 4 %.2hhx:%.2hhx:00:00:00:00 2>&1",
            param.csi_config.dev_password.c_str(), param.csi_config.dev_hostname.c_str(), param.csi_config.dev_ip.c_str(), param.csi_config.channel, param.csi_config.bw, param.csi_config.rx_mac_filter.mac[0],param.csi_config.rx_mac_filter.mac[1]);
    
  }
  else{
	sprintf(configcmd, "sshpass -p %s ssh -o strictHostKeyChecking=no %s@%s /jffs/csi/setup.sh %d %d 4 2>&1",             param.csi_config.dev_password.c_str(), param.csi_config.dev_hostname.c_str(), param.csi_config.dev_ip.c_str(), param.csi_config.channel, param.csi_config.bw);
  }
  printf("---RUN---\n %s\n",configcmd);
  printf("%s\n",sh_exec_block(configcmd).c_str());

  if(param.csi_config.beacon_rate > 0) {
    printf("starting tx...\n");
    
	sprintf(configcmd, "sshpass -p %s ssh -o strictHostKeyChecking=no %s@%s \"/jffs/csi/send.sh %d %d %d %s 11 11 11 %x %x %x  2>&1 & \"",
            param.csi_config.dev_password.c_str(), param.csi_config.dev_hostname.c_str(), param.csi_config.dev_ip.c_str(), param.csi_config.bw, param.csi_config.beacon_tx_streams,
            (int)param.csi_config.beacon_rate*1000, iface.c_str(), param.csi_config.beacon_mac_4, param.csi_config.beacon_mac_5, param.csi_config.beacon_mac_6);
	ROS_WARN("Beaconing on 11:11:11:%x:%x:%x\n",param.csi_config.beacon_mac_4,param.csi_config.beacon_mac_5,param.csi_config.beacon_mac_6);
    printf("---RUN---\n %s\n",configcmd);
    printf("%s\n",sh_exec_block(configcmd).c_str());
    
  }
  printf("done with config.\n");
}

void parse_csi(unsigned char* data, size_t nbytes, const nex_config_t& param){
  //8 is sizeof the ethernet header
  csi_udp_frame *rxframe = reinterpret_cast<csi_udp_frame*>(data);
  if(!(param.csi_config.rx_mac_filter.matches(rxframe->src_mac))) return;
  csi_instance out;

  out.rssi = rxframe->rssi;
  //ROS_INFO("%.4hhx", rxframe->kk1);
  memcpy(out.source_mac, rxframe->src_mac, 6);
  out.seq = rxframe->seqCnt;
  out.fc = (uint8_t)(rxframe->fc);

  out.tx = (rxframe->csiconf >> 11) & 0x3;
  out.rx = (rxframe->csiconf >> 8) & 0x3;
  out.bw = (rxframe->chanspec>>11) & 0x07;
  out.channel = (rxframe->chanspec) & 255;

  switch(out.bw)
    {
    case 0x4:
	  out.bw = 80;
	  break;
    case 0x3:
	  out.bw = 40;
	  break;
    case 0x2:
	  out.bw = 20;
	  break;
    default:
	  ROS_ERROR("Invalid Bandwidth received %d", out.bw);
	  return;
    }

  uint32_t n_sub = (uint32_t)(((float)out.bw) *3.2);
  size_t csi_nbytes = (size_t)(n_sub * sizeof(int32_t));

  uint32_t *csi = reinterpret_cast<uint32_t*>(data+sizeof(csi_udp_frame));

  out.n_sub = n_sub;
  out.csi_r.resize(n_sub);
  out.csi_i.resize(n_sub);

  //decode CSI
  uint64_t c_r, c_i;
  uint64_t c_r_buf[n_sub], c_i_buf[n_sub];
  for(int i = 0; i < n_sub; ++i){

	uint32_t c = (uint64_t)csi[i];
	c_r=0;
	c_i=0;
	uint32_t exp = ((int32_t)(c & e_mask) - 31 + 1023);
	uint32_t r_exp = exp;
	uint32_t i_exp = exp;

	uint32_t r_mant = (c&r_mant_mask) >> 18;
	uint32_t i_mant = (c&i_mant_mask) >> 6;

	//construct real mantissa
	uint32_t e_shift = 0;
	while(!(r_mant & count_mask)){
	  r_mant *= 2;
	  e_shift += 1;
	  if(e_shift == 10){
		r_exp = 1023;
		e_shift = 0;
		r_mant = 0;
		break;
	  }
	}
	r_exp -= e_shift;

	//construct imaginary mantissa
	e_shift = 0;
	while(!(i_mant & count_mask)){
	  i_mant *= 2;
	  e_shift += 1;
	  if(e_shift == 10){
		i_exp = 1023;
		e_shift = 0;
		i_mant = 0;
		break;
	  }
	}
	i_exp -= e_shift;

	//construct doubles
	c_r |= (uint64_t)(c & r_sign_mask) << 34;
	c_i |= (uint64_t)(c & i_sign_mask) << 46;

	c_r |= ((uint64_t)(r_mant & mant_mask)) << 42;
	c_i |= ((uint64_t)(i_mant & mant_mask)) << 42;

	c_r |= ((uint64_t)r_exp)<<52;
	c_i |= ((uint64_t)i_exp)<<52;

	//place doubles
	c_r_buf[i] = c_r;
	c_i_buf[i] = c_i;

  }

  //copy to struct
  memcpy(out.csi_r.data(), c_r_buf, sizeof(double)*n_sub);
  memcpy(out.csi_i.data(), c_i_buf, sizeof(double)*n_sub);

  //scan for repeated tx-rx
  bool new_csi = false;
  for(auto ch_it = g_mimo_channel.begin(); ch_it != g_mimo_channel.end(); ++ch_it){
	int out_comb = out.tx*4 + out.rx;
	if(ch_it->tx*4 + ch_it->rx == out_comb){
	  new_csi = true;
	}
  }
  if(out.seq != g_last_seq && g_mimo_channel.size() > 0){
	new_csi = true;
  }

  if(new_csi){
	publish_csi(g_mimo_channel);
	g_mimo_channel.clear();
  }
  g_last_seq = out.seq;
  //save the currently extracted CSI
  g_mimo_channel.push_back(out);
}




void wiros_main(nex_config_t &param){
  //check if remote device is active.
  contact_device(param);
  //handle shutdown
  signal(SIGINT, handle_shutdown);
  
  //change last byte of beacon mac to reflect our IP.
  size_t pos = g_host_ip.rfind('.');
  param.csi_config.beacon_mac_6 = (uint8_t)std::stoi(std::string(g_host_ip).erase(0,pos+1));

  //configure CSI collection on ASUS.
  configure_device(param);

  int sockfd, connfd;
  socklen_t len;

  struct sockaddr_in servaddr, cliaddr;
  socklen_t sockaddr_len = sizeof(cliaddr);

  // Create socket
  if (param.use_tcp_forward) {//forward over tcpdump-netcat
	if ( (sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0 ) {
	  perror("socket creation failed");
	  exit(EXIT_FAILURE);
	}
  }
  else{//forward over udp
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0 ) {
	  perror("socket creation failed");
	  exit(EXIT_FAILURE);
    }
	struct timeval tv;
	tv.tv_sec = 1;
	tv.tv_usec = 10000;
	if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO,&tv,sizeof(tv)) < 0) {
	  perror("Error");
	}
  }
  memset(&servaddr, 0, sizeof(servaddr));
  memset(&cliaddr, 0, sizeof(cliaddr));

  // Filling server information
  servaddr.sin_family    = AF_INET; // IPv4
  servaddr.sin_addr.s_addr = INADDR_ANY;
  if(param.use_tcp_forward)
    servaddr.sin_port = htons(PORT_TCP);
  else
    servaddr.sin_port = htons(PORT);

  //can't do &(1) in c++ so need to do this
  int yes=1;
  setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(int));
  setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int));
  setsockopt(sockfd, SOL_SOCKET, SO_REUSEPORT, &yes, sizeof(int));
  // Bind the socket with the server address
  if ( bind(sockfd, (const struct sockaddr *)&servaddr,
			sizeof(servaddr)) < 0 )
	{
	  ROS_INFO("bind failed: %s", strerror(errno));
	  exit(EXIT_FAILURE);
	}

  size_t n;
  //normal udp broadcast version, change to support tcp_forward.
  if(!param.use_tcp_forward){
    while(1){
      if(!g_ok){
        break;
      }
	  if ((n = recvfrom(sockfd, g_csi_buf.data(), CSI_BUF_SIZE, 0, (struct sockaddr *)&cliaddr, &sockaddr_len)) == -1){
		if(!(errno == ETIMEDOUT || errno == EAGAIN || errno == EINTR)){
          ROS_ERROR("Socket Error: %s", strerror(errno));
		}
        continue;
	  }
	  //	  ROS_INFO("%s", inet_ntoa(cliaddr.sin_addr));
	  if(n > 0){
		parse_csi(g_csi_buf.data(), n,param);
	  }
    }
  } else{
    printf("TCP forwarding is unimplemented :( Ask william to implement it.\n");
    exit(1);
  }

}
