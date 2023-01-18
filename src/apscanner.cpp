#include <ros/ros.h>
#include "shutils.h"
#include <vector>
#include <stdio.h>
#include <regex>
#include <sstream>
#include <iomanip>
#include <rf_msgs/AccessPoints.h>

const std::regex addr_ex("(..):(..):(..):(..):(..):(..)");
const std::regex channel_ex("Channel:(\\d+)");
const std::regex rssi_ex("Signal level=([\\-0-9]+) dBm");

ros::Publisher pub_5;
ros::Publisher pub_2;


int main(int argc, char* argv[]){
  ros::init(argc, argv, "ap_scanner", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh("~");
  std::string iface;
  std::string topic;
  double scan_period;
  nh.param<std::string>("iface", iface);
  nh.param<std::string>("topic", topic, "aps");
  nh.param<double>("period", scan_period, 20.0);

  char topic2[64];
  sprintf(topic2,"%s_2",topic.c_str());
  char topic5[64];
  sprintf(topic5,"%s_5",topic.c_str());
  
  pub_5 = nh.advertise<rf_msgs::AccessPoints>(topic5,3);
  pub_2 = nh.advertise<rf_msgs::AccessPoints>(topic2,3);

  char scan_cmd[128];
  sprintf(scan_cmd,"sudo iwlist %s scanning | egrep --color=never 'Address|Signal level|Channel:' 2>&1",iface.c_str());
  
  while(ros::ok()){
	std::string s = sh_exec_block(scan_cmd);
	//	  std::string s = sh_exec_block("echo 'Cell:50   Cell Cell  Cell   47 Cell\nCell'");
	if(s.find("Interface doesn't support") != std::string::npos){
	  sleep(0.1);
	  continue;
	}
	
	rf_msgs::AccessPoints aps_5;
	rf_msgs::AccessPoints aps_2;
	
	size_t s_start = s.find("Cell",0);
	size_t s_end = s_start;
	while((s_end=s.find("Cell", s_start+1)) != std::string::npos){
	  std::string entry = s.substr(s_start, s_end-s_start);
	  ROS_INFO("%s",entry.c_str());
	  std::smatch mac_match;
	  std::smatch chan_match;
	  std::smatch rssi_match;
	  
	  rf_msgs::Station ap;
	  if(std::regex_search(entry,mac_match,addr_ex)){
		ROS_INFO("%s",mac_match[0].str().c_str());
		for(int i = 1; i < mac_match.size(); ++i){
		  ap.mac[i-1] = std::strtol(mac_match[i].str().c_str(), NULL, 16);
		  ROS_INFO("%s:%d", mac_match[i].str().c_str(),ap.mac[i-1]);
		}
	  }
	  else continue;
	  if(std::regex_search(entry,chan_match,channel_ex)){
		ap.channel = std::stoi(chan_match[1].str());
		ROS_INFO("%d", ap.channel);
	  }
	  else continue;
	  if(std::regex_search(entry,rssi_match, rssi_ex)){
		ap.rssi = std::stoi(rssi_match[1].str());
		ROS_INFO("%d", ap.rssi);
	  }
	  else continue;

	  if(ap.channel <= 14){
		aps_2.aps.push_back(ap);
	  }
	  else{
		aps_5.aps.push_back(ap);
	  }
	  
	  s_start = s_end;
	}
	std::sort(aps_5.aps.begin(), aps_5.aps.end(), [](rf_msgs::Station a1, rf_msgs::Station a2){
											return a1.rssi > a2.rssi;
										  }
	  );
	
	pub_5.publish(aps_5);
	pub_2.publish(aps_2);

    ros::Duration(scan_period).sleep();
    ROS_INFO("time is %f", ros::Time::now().toSec());
  }
}

