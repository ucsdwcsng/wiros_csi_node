#include <nex_conf.hpp>
#include <shell_utils.hpp>
#include <ros_interface.hpp>
#include <utils.h>

//forward decl of local functions
void handle_shutdown(int sig);
nex_config_t setup_params(ros::NodeHandle& nh);
void contact_device(nex_config_t& params);

int main(int argc, char* argv[]){
  nex_config_t params;
  
  //setup ROS
  params = ros_initialize(argc, argv);
  //shutdown handler
  signal(SIGINT, handle_shutdown);

  //display starting chanspec
  ROS_INFO("chanspec %d/%d", (int)params.csi_config.ch, (int)params.csi_config.bw);

  //Find the ASUS if its IP has not been specified, and try to contact it.
  contact_device(params);

  
}

void handle_shutdown(int sig){
  ROS_WARN("Shutting down.");
  if(cli_fp){
	ROS_WARN("Closing tcpdump process");
	pclose(cli_fp);
  }
  if(tx_fp){
	ROS_WARN("Closing tx process");
	pclose(tx_fp);
	char killcmd[128];
	
	sprintf(killcmd, "sshpass -p %s ssh -o strictHostKeyChecking=no %s@%s killall send.sh", rx_pass.c_str(), rx_host.c_str(), rx_ip.c_str());
	sh_exec(std::string(killcmd));
  }
  ROS_WARN("Calling ros::shutdown()");
  ros::shutdown();
  ROS_WARN("Done.");
}

void contact_device(nex_config_t& params){

  //automatically find connected asus router
  std::smatch ip_match;
  char subnet[20];
  bool scan=false;
  if(std::regex_search(rx_ip,ip_match,ip_regex)){
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
  std::string hostIP;
  bool iface_up = false;
  while(getline(IPs, IP, ' ')){
	if (IP.rfind(subnet, 0) == 0) {
	  hostIP = IP;
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
			if(temp_ip != hostIP){
			  rx_ip = temp_ip;
			}
		  }
		}
	  }
	}
  }
  char setupcmd[512];
  sprintf(setupcmd, "ping -c 3 -i 0.3 %s", rx_ip.c_str());
  std::string ping_result = sh_exec_block(setupcmd);
  ROS_INFO("%s", ping_result.c_str());
  if(ping_result.find("Destination Host Unreachable",0) != std::string::npos || ping_result.find("100% packet loss",0) != std::string::npos){
	ROS_ERROR("The host at %s did not respond to a ping.", rx_ip.c_str());
	ROS_ERROR("This is probably because the 'asus_ip' param is setup to the incorrect value.");
	ROS_ERROR("You can enable automatic ASUS detection by setting 'asus_ip' to \"\"");
	exit(1);
  }
}
