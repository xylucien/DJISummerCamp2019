//
// Created by Abiel on 7/31/19.
//

#ifndef MANIFOLDCAN_MANIFOLDCAN_H
#define MANIFOLDCAN_MANIFOLDCAN_H

#include <linux/can.h>
#include <linux/can/raw.h>

#include <string>
#include <libnet.h>
#include <mutex>
#include <fcntl.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include "CANTopic.h"

class ManifoldCAN {
public:
    ManifoldCAN(const std::string &canInterface);

    void threadUpdate();

private:
    std::string canInterfaceName;
    int canSocket;

    struct sockaddr_can addr;
    struct ifreq ifr;
};


#endif //MANIFOLDCAN_MANIFOLDCAN_H
