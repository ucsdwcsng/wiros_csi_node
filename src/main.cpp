#include <nex_conf.hpp>
#include <shell_utils.hpp>

//forward decl of local functions
void handle_shutdown(int sig);
nex_config_t setup_params(ros::NodeHandle& nh);

int main(int argc, char* argv[]){
  nex_config_t params;
  
  //setup ROS
  params = ros_initialize(argc, argv);
  //shutdown handler
  signal(SIGINT, handle_shutdown);

  autodetect_device(
  
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

