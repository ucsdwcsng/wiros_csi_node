//reads udp data from nexmon_csi running on an ASUS
//converts to CSI message format

#include "nexcsiserver.h"

//CSI-Buffering related globals
std::vector<csi_instance> channel_current;
uint16_t last_seq;

//Buffer in which the full CSI message is reconstructed
double* csi_r_out = NULL;
double* csi_i_out = NULL;
//size of the above buffers
size_t csi_size = 0;

//Info about the ASUS that the node is connected to
std::string rx_ip;
std::string rx_pass;
char* rx_host;

//Forward the packets over tcpdump->netcat (older kernels won't receive the udp broadcasts)
bool use_tcp = false;

//Don't configure
bool no_config = false;

//Info about the node itself
std::string hostname;

//publisher
ros::Publisher pub_csi;
ros::Subscriber sub_ap;

//info about current wireless settings
int ch, bw;
double beacon;

//MAC addresses to be filtered for in software
std::vector<int> mac_filter;
bool use_software_mac_filter = true;

//if not "", the node will listen on the given topic for a list of APs and try to collect CSI from
//the first AP on the list.
std::string lock_topic;

//various buffers
unsigned char *csi_buf, *csi_data;

int main(int argc, char* argv[]){

//setup ros
    ros::init(argc, argv, "nexcsi", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh("~");

    ros::AsyncSpinner spinner(0);
    spinner.start();

    pub_csi = nh.advertise<rf_msgs::Wifi>("/csi",10);
    ros::ServiceServer set_chanspec_srv = nh.advertiseService<nexmon_csi_ros::ConfigureCSI::Request, nexmon_csi_ros::ConfigureCSI::Response>("configure_csi",config_csi_callback);

    ROS_INFO("Publishing: %s", pub_csi.getTopic().c_str());

//handle shutdown
    signal(SIGINT, handle_shutdown);

//read params
    setup_params(nh);

//optional subscribe to AP info topic
    if(lock_topic != std::string("")){
        sub_ap = nh.subscribe(lock_topic, 10, ap_info_callback);
        ROS_INFO("Subscribing: %s", sub_ap.getTopic().c_str());
    }

//display starting chanspec
    ROS_INFO("chanspec %d/%d", (int)ch, (int)bw);

//automatically find connected asus router
    std::smatch ip_match;
    char subnet[20];
    bool scan=false;
    if(std::regex_search(rx_ip,ip_match,ip_ex)){
        sprintf(subnet, "%s.%s.%s.", ip_match[1].str().c_str(), ip_match[2].str().c_str(), ip_match[3].str().c_str());
        if(ip_match[4]=="*"){  
            scan=true;
        }
    }
    else{
        ROS_FATAL("Invalid target IP, needs to be xxx.xxx.xxx.xxx or xxx.xxx.xxx.*");
    }

    std::stringstream IPs(sh_exec("hostname -I"));
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
                std::stringstream nmap(sh_exec(cmd));
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

    if(!iface_up){
        ROS_ERROR("The subnet does not appear to be active.");
        exit(1);
    }

//figure out the name of this computer
    hostname = sh_exec("hostname");
//mac address we will transmit on will be 17:17:17:first byte of name:second byte of name:last byte of ip4
    uint8_t mac4 = hostname[0];
    uint8_t mac5 = hostname[1];
    size_t pos = hostIP.rfind('.');
    uint8_t mac6 = (uint8_t)std::stoi(std::string(hostIP).erase(0,pos+1));

    if(!ros::ok()){
        return 0;
    }

    if(!iface_up){
        ROS_INFO("The connection w/subnet %s is not active.", subnet);
        exit(1);
    }



//configure the receiver
    ROS_INFO("Configuring Receiver...");

    if(no_config){ROS_WARN("Not configuring the router.");}
    else{
        bool unconfigured = true;
        while(unconfigured){
            if(set_chanspec((int)ch, (int)bw)){
                ROS_ERROR("Invalid channel or bandwidth.");
                exit(1);
            }
            std::string res_out = reconfigure();
            ROS_INFO("\n***\nSetup Output:\n\n%s\n***", res_out.c_str());
            if(res_out.find("Permission denied") != std::string::npos){
                ROS_ERROR("A device was found at %s, but it refused SSH access.\nPlease check the 'asus_pwd' param and ensure it is set to the device's password.\nCurrent passsword: %s\nThis may also be caused by setup scripts not having the correct permissions set.",rx_ip.c_str(),rx_pass.c_str());
                exit(EXIT_FAILURE);
            }
            if(res_out.find("Connection refused") != std::string::npos){
                ROS_INFO("Waiting 5 seconds and retrying...");
                sleep(5);
            }
            else{
                unconfigured = false;
            }
        }
    }

    if(beacon > 0) {
        ROS_INFO("Starting transmitter...");
        sprintf(setupcmd, "sshpass -p %s ssh -o strictHostKeyChecking=no %s@%s /jffs/csi/send.sh 80 4 %d 11 11 11 %x %x %x > /dev/null 2>&1",
            rx_pass.c_str(), rx_host, rx_ip.c_str(),
            (int)beacon*1000, mac4, mac5, mac6);
        ROS_INFO("%s", setupcmd);
        tx_fp = popen(setupcmd, "r");
    }

    int sockfd, connfd;
    socklen_t len;

    struct sockaddr_in servaddr, cliaddr;
    socklen_t sockaddr_len = sizeof(cliaddr);

// Create socket
if (use_tcp) {//forward over tcpdump-netcat
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
}
memset(&servaddr, 0, sizeof(servaddr));
memset(&cliaddr, 0, sizeof(cliaddr));

// Filling server information
servaddr.sin_family    = AF_INET; // IPv4
servaddr.sin_addr.s_addr = INADDR_ANY;
if(use_tcp)
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

ROS_INFO("Opened socket.");

int n;

if(use_tcp){
    setup_tcpdump(hostIP);
    while(ros::ok() && (listen(sockfd, 5)) != 0){
        ROS_INFO("Waiting for TCP connection...");
        sleep(1);
    }
    while(ros::ok()) {
        connfd = accept(sockfd, (SA * ) & cliaddr, &len);
        if (connfd == -1 && errno == EAGAIN) {
            sleep(0.05);
            continue;
        }
        else if(connfd > 0) {
            ROS_INFO("Accepted Connection from %s", rx_ip.c_str());
            break;
        }
        else{
            ROS_ERROR("Connection failed.");
            exit(1);
        }
    }
}

ROS_INFO("Starting CSI collection");
//buffer to hold incoming UDP data
char buffer[MAXLINE];

//incoming data is separated by packet, stored in here
csi_buf = new unsigned char[CSI_BUF_SIZE];
csi_data = new unsigned char[CSI_BUF_SIZE];
//CSI binary data is then processed into a csi_data struct, put in here
std::queue<csi_instance> pkts;
int it = 0;
size_t csi_pos = 0;

ROS_WARN("Filtering for MAC Addresses: %s",hr_mac_filt(mac_filter).c_str());

//normal udp broadcast version
if(!use_tcp){
    while(ros::ok()){
        if ((n = recvfrom(sockfd, csi_buf, CSI_BUF_SIZE, 0, (struct sockaddr *)&cliaddr, &sockaddr_len)) == -1){
            ROS_ERROR("Socket Error: %s", strerror(errno));
        }

        if(n > 0){
            parse_csi(csi_buf, n);
        }
    }
}

else{//decode the received packet from tcpdump
    while(ros::ok()){

        n = read(connfd, buffer, MAXLINE);
        if(n == 0) continue;

//if too much data is accumulating with no packets found, get rid of all our data
        if(csi_pos + n > CSI_BUF_SIZE){
            ROS_ERROR("CSI BUFFER OVERFLOWED, FLUSHING");
            memset(csi_buf, 0, CSI_BUF_SIZE);
            csi_pos = 0;
        }

//add new data to buffer
        memcpy(csi_buf + csi_pos, buffer, n);
        csi_pos += n;
        size_t hdr_pos = 0;
//searches for start of the next packet from first 8 bytes (as we expect the beginning of the buffer to be the header of the packet currently being received)
        while((hdr_pos = find_csi_hdr(csi_buf)) != NO_HDR_IN_BUF){
            memcpy(csi_data, csi_buf, hdr_pos);
//size of remaining data in buffer
            csi_pos = csi_pos - hdr_pos;
//shift data back in buffer
            memcpy(csi_buf, csi_buf + hdr_pos, csi_pos);
//now process csi data
            parse_csi(csi_data+8+8, hdr_pos);
//clear empty space in the buffer
            memset(csi_buf + csi_pos, 0, CSI_BUF_SIZE - csi_pos);
        }
    }
}
}

void parse_csi(unsigned char* data, size_t nbytes){
//8 is sizeof the ethernet header
    csi_udp_frame *rxframe = reinterpret_cast<csi_udp_frame*>(data);
    if(use_software_mac_filter){
        for(int i = 0; i < mac_filter.size(); ++i){
            if(rxframe->src_mac[i] != (uint8_t)mac_filter[i]){
// ROS_WARN("Rejected in software");
                return;
            }
        }
    }
    csi_instance out;

    out.rssi = rxframe->rssi;
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
    out.csi_r = new double[n_sub];
    out.csi_i = new double[n_sub];

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
    memcpy(out.csi_r, c_r_buf, sizeof(double)*n_sub);
    memcpy(out.csi_i, c_i_buf, sizeof(double)*n_sub);

//scan for repeated tx-rx
    bool new_csi = false;
    for(auto ch_it = channel_current.begin(); ch_it != channel_current.end(); ++ch_it){
        int out_comb = out.tx*4 + out.rx;
        if(ch_it->tx*4 + ch_it->rx == out_comb){
            new_csi = true;
        }
    }
    if(out.seq != last_seq && channel_current.size() > 0){
        new_csi = true;
    }

    if(new_csi){
        publish_csi(channel_current);
        channel_current.clear();
    }

#ifdef DEBUG_CSI
    ROS_WARN("%s: seq %d fc %.2hhx rx %d tx %d, csi %zu bytes.", hr_mac(out.source_mac).c_str(), out.seq, out.fc, out.rx, out.tx, csi_nbytes);
#endif

    last_seq = out.seq;
//save the currently extracted CSI
    channel_current.push_back(out);
}

void publish_csi(std::vector<csi_instance> &channel_current){
//4x4 matrices, with n_sub elements each, w/ interleaved 4 byte real + imag parts
    csi_instance csi_0 = channel_current.at(0);
    size_t rx_stride = csi_0.n_sub;
    size_t rx2 = rx_stride / 2;
    size_t tx_stride = rx_stride*4;
    size_t num_floats = tx_stride*4;
    rf_msgs::Wifi msgout;
    if(num_floats > csi_size){
        delete csi_r_out;
        delete csi_i_out;
//statically allocate csi_out for now
        csi_r_out = new double[num_floats];
        csi_i_out = new double[num_floats];
        csi_size = num_floats;
    }
    msgout.header.stamp = ros::Time::now();
    msgout.ap_id = 0;
    msgout.txmac = std::vector<unsigned char>(csi_0.source_mac, csi_0.source_mac + 6);
    msgout.chan = csi_0.channel;
    msgout.n_sub = csi_0.n_sub;
    msgout.seq_num = csi_0.seq;
    msgout.fc = csi_0.fc;
    msgout.n_rows = 4;
    msgout.n_cols = 4;
    msgout.bw = csi_0.bw;
    msgout.mcs = 0;
    msgout.rssi = (int32_t)(csi_0.rssi);
    ROS_INFO("%s: RSSI %d, seq %d, fc %.2hhx, rx %s",hr_mac(csi_0.source_mac).c_str(), msgout.rssi, msgout.seq_num, csi_0.fc, rx_ip.c_str());
    memset(csi_r_out,0,num_floats*sizeof(double));
    memset(csi_i_out,0,num_floats*sizeof(double));
    msgout.rx_id = rx_ip;
    msgout.msg_id = 0;
    for(auto c = channel_current.begin(); c != channel_current.end(); ++c){
        size_t csi_idx = rx_stride*c->rx + tx_stride*c->tx;
//fft-shift the data
        for(int bin = 0; bin < rx2; ++bin){
            csi_r_out[csi_idx + bin + rx2] = (c->csi_r[bin]);
            csi_i_out[csi_idx + bin + rx2] = (c->csi_i[bin]);
            csi_r_out[csi_idx + bin] = (c->csi_r[bin + rx2]);
            csi_i_out[csi_idx + bin] = (c->csi_i[bin + rx2]);
        }
    }
    msgout.csi_real = std::vector<double>(csi_r_out, csi_r_out + num_floats);
    msgout.csi_imag = std::vector<double>(csi_i_out, csi_i_out + num_floats);
    pub_csi.publish(msgout);
}

void handle_shutdown(int sig){
    if(cli_fp){
        pclose(cli_fp);
    }
    if(tx_fp){
        pclose(tx_fp);
        char killcmd[128];
        sprintf(killcmd, "sshpass -p %s ssh -o strictHostKeyChecking=no %s@%s killall send.sh", rx_pass.c_str(), rx_host, rx_ip.c_str());
        sh_exec(std::string(killcmd));
    }
    ros::shutdown();
}

//returns true on error
bool set_chanspec(int s_chan, int s_bw){
    if(!(s_bw == -1 || s_bw ==20 || s_bw == 40 || s_bw == 80))
        return true;
    if(s_chan != -1 && s_chan != ch){
        ch = s_chan;
        ROS_WARN("Setting CHANNEL to %d", ch);
    }
    if(s_bw != -1 && s_bw != bw){
        bw = s_bw;
        ROS_WARN("Setting BW to %d", bw);
    }
    return false;
}

bool set_mac_filter(std::vector<int> filt){
    if(mac_filter.size() == 0){
        mac_filter = filt;
        return true;
    }
    if(mac_filter.size() > 6){
        return true;
    }
    if(mac_filter[0] == -1){
        return true;
    }
    if(mac_filter == filt){
        return true;
    }
    mac_filter = filt;
    ROS_WARN("Set MAC FILTER to %s",hr_mac_filt(mac_filter).c_str());
    if(mac_filter.size() == 2){
        use_software_mac_filter = false;
    }
    return false;
}

bool set_mac_filter(const uint8_t* mac){
    std::vector<int> nmac(mac, mac+6);
    if(nmac == mac_filter){
        ROS_INFO("Already locked on to %s", hr_mac(mac).c_str());
        return true;
    }
    mac_filter = nmac;
    ROS_WARN("Set MAC FILTER to %s",hr_mac_filt(mac_filter).c_str());
    return false;
}


std::string reconfigure(){
    char configcmd[512];
    if(mac_filter.size() > 1){
        sprintf(configcmd, "sshpass -p %s ssh -o strictHostKeyChecking=no %s@%s /jffs/csi/setup.sh %d %d 4 %.2hhx:%.2hhx:00:00:00:00 2>&1",
            rx_pass.c_str(), rx_host, rx_ip.c_str(), ch, bw, mac_filter[0],mac_filter[1]);
    }
    else{
        sprintf(configcmd, "sshpass -p %s ssh -o strictHostKeyChecking=no %s@%s /jffs/csi/setup.sh %d %d 4 2>&1", rx_pass.c_str(), rx_host, rx_ip.c_str(), ch, bw);
    }
    ROS_INFO("%s",configcmd);
    return sh_exec_block(configcmd);
}

void setup_tcpdump(std::string hostIP){
    char setupcmd[512];
    sprintf(setupcmd, "sshpass -p %s ssh -o strictHostKeyChecking=no %s@%s /jffs/csi/tcpdump -i eth6 port 5500 -nn -s 0 -w - --immediate-mode | nc %s %d > /dev/null 2>&1", rx_pass.c_str(), rx_host, rx_ip.c_str(), hostIP.c_str(), PORT_TCP);
    ROS_INFO("%s",setupcmd);
    cli_fp = popen(setupcmd, "r");
}



void setup_params(ros::NodeHandle& nh){
    double tmp_ch, tmp_bw;
    nh.param<double>("channel", tmp_ch, 157.0);
    nh.param<double>("bw", tmp_bw, 80.0);
    ch = (int)tmp_ch;
    bw = (int)tmp_bw;
	std::string tmp_host;
    nh.param<double>("beacon_rate", beacon, 200.0);
    nh.param<bool>("tcp_forward", use_tcp, false);
    nh.param<std::string>("asus_ip", rx_ip, "");
    nh.param<std::string>("asus_pwd", rx_pass, "password");
    nh.param<bool>("no_config", no_config, false);
    nh.param<std::string>("lock_topic", lock_topic, "");
	nh.param<std::string>("asus_host", tmp_host, "HOST");
	rx_host = (char*)tmp_host.c_str();

//MAC filter param
    std::string mac_filter_temp;
    nh.param<std::string>("mac_filter", mac_filter_temp, std::string(""));
    set_mac_filter(mac_filter_strtovec(mac_filter_temp));
// for(auto i=mac_filter.begin(); i != mac_filter.end(); ++i){
//    ROS_INFO("%d", (*i));
// }
}


//handle change of channel, returns false on error.
bool config_csi_callback(nexmon_csi_ros::ConfigureCSI::Request &req, nexmon_csi_ros::ConfigureCSI::Response &resp){
    if(req.chan == ch && req.bw == bw && req.mac_filter == mac_filter){
        resp.result = "No Change Applied.";
        return true;
    }
    if(set_chanspec(req.chan, req.bw)){
        resp.result = "Error: Invalid Channel Or Bandwidth";
        return false;
    }
    if(set_mac_filter(req.mac_filter)){
        resp.result = "Error: Invalid MAC Filter";
        return false;
    }
    resp.result = reconfigure();
    resp.result.erase(std::remove_if(resp.result.begin(),resp.result.end(), sanitize_string), resp.result.end());
    return true;
}

void ap_info_callback(const rf_msgs::AccessPoints::ConstPtr& msg){
    uint8_t a_mac[6];
    memcpy(a_mac, msg->aps[0].mac.data(), 6);
    if(!set_mac_filter(a_mac) && !set_chanspec(msg->aps[0].channel, 20)){
        reconfigure();
    }
}