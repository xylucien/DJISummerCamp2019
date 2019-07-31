//
// Created by Abiel on 7/31/19.
//

#include "ManifoldCAN.h"
#include <stdexcept>
#include <iostream>
#include <cstring>

ManifoldCAN::ManifoldCAN(const std::string &canInterface) {
    this->canInterfaceName = canInterface;

    canSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    if(canSocket < 0){
        throw std::runtime_error("Unable to start CAN socket errno: " + std::to_string(errno));
    }

    struct can_filter filter;
    filter.can_id   = 0x666;
    filter.can_mask = CAN_SFF_MASK;

    /*
    int sockOptRet = setsockopt(
            canSocket,
            SOL_CAN_RAW,
            CAN_RAW_FILTER,
            &filter,
            sizeof(filter)
    );
     */

    //if(sockOptRet < 0){
    //    throw std::runtime_error("setsockopt failed with: " + std::to_string(errno));
    //}

    strcpy(ifr.ifr_name, canInterface.c_str());
    ioctl(canSocket, SIOCGIFINDEX, &ifr);

    struct sockaddr_can addr;

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    int bindRet = bind(canSocket, (struct sockaddr*) (&addr), sizeof(addr));

    if(bindRet < 0){
        throw std::runtime_error("Bind failed with: " + std::to_string(errno));
    }

    /*
    strcpy(ifr.ifr_name, canInterface.c_str());
    ioctl(canSocket, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if(connect(canSocket, (struct sockaddr*) &addr, sizeof(addr)) < 0){
        throw std::runtime_error("Unable to connect to CAN interface errno: " + std::to_string(errno));
    }


    rxMsg.msg_head.opcode  = RX_SETUP;
    rxMsg.msg_head.can_id  = 0x666;
    rxMsg.msg_head.flags   = 0;
    rxMsg.msg_head.nframes = 0;

    //if(write(canSocket, &rxMsg, sizeof(rxMsg)) < 0){
    //    throw std::runtime_error("RX setup error: " + std::to_string(errno));
    //}
     */
}

void ManifoldCAN::threadUpdate() {
    //TODO finish this
    canfd_frame frame;

    ssize_t size = read(canSocket, &frame, CANFD_MTU);

    if(size <= 0){
        std::cout << "a" << std::endl;
        //throw std::runtime_error("CAN returned zero bytes!");
    }

    long inputBuf = 0;
    inputBuf |= ((long)frame.data[0]) << 24;
    inputBuf |= ((long)frame.data[1]) << 16;
    inputBuf |= ((long)frame.data[2]) << 8;
    inputBuf |= ((long)frame.data[3]);
    float input = *(float*) &inputBuf;

    inputBuf = 0;
    inputBuf |= ((uint8_t)frame.data[4]) << 24;
    inputBuf |= ((uint8_t)frame.data[5]) << 16;
    inputBuf |= ((uint8_t)frame.data[6]) << 8;
    inputBuf |= ((uint8_t)frame.data[7]);

    //uint16_t ret = be16toh(*(std::uint16_t *)(frame.data + 0));

    //uint8_t id;
    //memcpy(&id, &frame.data, sizeof(uint8_t));
    //memcpy(&input, &frame.data + 8, 4);

    std::cout << "ID: " << std::to_string(frame.can_id) << " Data: " << input << " message ID: " << inputBuf << std::endl;
}

