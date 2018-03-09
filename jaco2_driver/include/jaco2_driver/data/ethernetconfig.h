#ifndef ETHERNETCONFIG_H
#define ETHERNETCONFIG_H
#include <string>
struct EthernetConfig{
    int local_cmd_port;
    int local_bcast_port;
    std::string local_ip_address;
    std::string robot_ip_address;
    std::string subnet_mask;
};

#endif // ETHERNETCONFIG_H
