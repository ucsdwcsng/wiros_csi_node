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
#include <iostream>

#include "nex_conf.hpp"
#include "shell_utils.hpp"
#include "nex_data.hpp"

#define CSI_BUF_SIZE 4096
#define PORT 5500
#define PORT_TCP 50005

volatile bool g_ok = true;
std::string g_host_ip = "";
std::vector<csi_instance> g_mimo_channel;
uint16_t g_last_seq;

void handle_shutdown(int sig)
{
    g_ok = false;
}

std::array<uint8_t, CSI_BUF_SIZE> g_csi_buf;

int contact_device(nex_config_t &params)
{
    std::smatch ip_match;
    char subnet[20];
    bool scan = false;
    if (std::regex_search(params.csi_config.dev_ip, ip_match, ip_ex))
    {
        sprintf(subnet, "%s.%s.%s.", ip_match[1].str().c_str(), ip_match[2].str().c_str(), ip_match[3].str().c_str());
        if (ip_match[4] == "*")
        {
            scan = true;
        }
        else
        {
            params.csi_config.beacon_mac_6 = (uint8_t)std::stoi(ip_match[4].str());
        }
    }
    else
    {
        std::cerr << "Invalid target IP, needs to be xxx.xxx.xxx.xxx or xxx.xxx.xxx.*" << std::endl;
        return 1;
    }

    std::stringstream IPs(sh_exec_block("hostname -I"));
    std::string IP;
    while (getline(IPs, IP, ' '))
    {
        if (IP.rfind(subnet, 0) == 0)
        {
            g_host_ip = IP;
        }
    }
    if (scan)
    {
        char cmd[256];
        std::cout << "Scanning for ASUS routers..." << std::endl;
        sprintf(cmd, "nmap -sP %s0/24", subnet);
        std::cout << cmd << std::endl;
        std::stringstream nmap(sh_exec_block(cmd));
        std::string target;
        while (getline(nmap, target, ' '))
        {
            if (target.rfind(subnet, 0) == 0)
            {
                std::string temp_ip = target.substr(0, target.find("\n"));
                if (temp_ip != g_host_ip)
                {
                    params.csi_config.dev_ip = temp_ip;
                    size_t pos = g_host_ip.rfind('.');
                    params.csi_config.beacon_mac_6 = (uint8_t)std::stoi(std::string(g_host_ip).erase(0, pos + 1));
                    std::cout << "Found AP at " << temp_ip << std::endl;
                }
            }
        }
    }

    char setupcmd[512];
    sprintf(setupcmd, "ping -c 3 -i 0.3 %s", params.csi_config.dev_ip.c_str());
    std::string ping_result = sh_exec_block(setupcmd);
    std::cout << ping_result << std::endl;
    if (ping_result.find("Destination Host Unreachable", 0) != std::string::npos || ping_result.find("100% packet loss", 0) != std::string::npos)
    {
        std::cerr << "The host at " << params.csi_config.dev_ip << " did not respond to a ping." << std::endl;
        std::cerr << "This is probably because the 'asus_ip' param is setup to the incorrect value." << std::endl;
        std::cerr << "You can enable automatic ASUS detection by setting 'asus_ip' to \"\"" << std::endl;
        return 1;
    }
    return 0;
}

int main(int argc, char **argv)
{
    signal(SIGINT, handle_shutdown);
    signal(SIGTERM, handle_shutdown);

    nex_config_t config;
    if (configure_device(config))
    {
        std::cerr << "Failed to configure device." << std::endl;
        return 1;
    }

    if (contact_device(config))
    {
        std::cerr << "Failed to contact device." << std::endl;
        return 1;
    }

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0)
    {
        std::cerr << "Could not open socket." << std::endl;
        return 1;
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        std::cerr << "Bind failed." << std::endl;
        return 1;
    }

    std::cout << "Listening for CSI packets on port " << PORT << std::endl;

    while (g_ok)
    {
        socklen_t addrlen = sizeof(addr);
        int bytes = recvfrom(sockfd, g_csi_buf.data(), CSI_BUF_SIZE, 0, (struct sockaddr *)&addr, &addrlen);
        if (bytes > 0)
        {
            parse_csi(g_csi_buf.data(), bytes, config);
        }
    }

    std::cout << "Shutting down." << std::endl;
    close(sockfd);

    return 0;
}
