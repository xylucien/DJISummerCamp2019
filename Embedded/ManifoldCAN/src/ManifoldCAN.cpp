//
// Created by Abiel on 7/31/19.
//

#include "ManifoldCAN.h"
#include "CANUtil.h"
#include <stdexcept>
#include <iostream>
#include <cstring>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

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
        throw std::runtime_error("CAN returned zero bytes!");
    }

    int messageId = deserializeInt(frame.data);
    float input = deserializeFloat(frame.data + 4);

    std::cout << "ID: " << std::hex << frame.can_id << " Data: " << input << " Message ID: " << messageId << std::endl;
}

void ManifoldCAN::writeTest() {
    canfd_frame frame;

    frame.can_id = 1638; //0x666
    frame.len = 8;

    for(float i = -1.0; i <= 1.0; i += .01){
        float testData = i;
        serializeInt(1, frame.data);
        serializeFloat(testData, frame.data + 4);

        write(canSocket, &frame, sizeof(struct can_frame));

        std::this_thread::sleep_for(1s);
    }
}