#include <string>

typedef struct csi_config{
  int channel;
  int bw;
  double beacon_rate;
  int beacon_tx_streams;
  std::string dev_ip;
  std::string dev_password;
  std::string dev_hostname;
  std::string rx_mac_filter;
} csi_config_t;

std::string apply_csi_config(csi_config_t conf){
  std::string iface;
  if(conf.channel >= 32){
      iface = "eth6";
      if(conf.beacon_tx_streams > 4) conf.beacon_tx_streams = 4;
  }
  else{
      iface = "eth5";
      if(conf.beacon_tx_streams > 3) conf.beacon_tx_streams = 3;
  }
  char configcmd[384];
  if(filter.len > 1){
	sprintf(configcmd, "sshpass -p %s ssh -o strictHostKeyChecking=no %s@%s /jffs/csi/setup.sh %d %d 4 %.2hhx:%.2hhx:00:00:00:00 2>&1",
            rx_pass.c_str(), rx_host.c_str(), rx_ip.c_str(), ch, bw, filter.mac[0],filter.mac[1]);
  }
  else{
	sprintf(configcmd, "sshpass -p %s ssh -o strictHostKeyChecking=no %s@%s /jffs/csi/setup.sh %d %d 4 2>&1", rx_pass.c_str(), rx_host.c_str(), rx_ip.c_str(), ch, bw);
  }

  

  
}
