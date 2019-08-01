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

int ManifoldCAN::sendFloatMessage(const FloatCANMessage &message) {
    canfd_frame frame;

    frame.can_id = 1638; //0x666
    frame.len = 8;

    serializeInt(message.id, frame.data);
    serializeFloat(message.data, frame.data + 4);

    return write(canSocket, &frame, sizeof(struct can_frame));
}

FloatCANMessage ManifoldCAN::receiveFloatMessage() const {
    canfd_frame frame;

    ssize_t size = read(canSocket, &frame, CANFD_MTU);

    if(size <= 0){
        throw std::runtime_error("CAN returned zero bytes!");
    }

    FloatCANMessage message;

    message.id = deserializeInt(frame.data);
    message.data = deserializeFloat(frame.data + 4);

    return message;
}

void ManifoldCAN::sendTargetVelocity(const Twist2D &twist) {

    int ret = sendFloatMessage(FloatCANMessage(CANMESSAGE_ID_TEST, twist.vX));

    ret = sendFloatMessage(FloatCANMessage(CANMESSAGE_ID_TARGET_VX, twist.vX));
    std::this_thread::sleep_for(1s);
    ret = sendFloatMessage(FloatCANMessage(CANMESSAGE_ID_TARGET_VY, twist.vY));
    std::this_thread::sleep_for(1s);
    ret = sendFloatMessage(FloatCANMessage(CANMESSAGE_ID_TARGET_VW, twist.w));
    std::this_thread::sleep_for(1s);

    //Some non zero number
    ret = sendFloatMessage(FloatCANMessage(CANMESSAGE_ID_TARGET_READY, 1.0f));
    std::this_thread::sleep_for(1s);
}

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
    rxMsg.msg_head.opcode  = RX_SETUP;
    rxMsg.msg_head.can_id  = 0x666;
    rxMsg.msg_head.flags   = 0;
    rxMsg.msg_head.nframes = 0;

    if(write(canSocket, &rxMsg, sizeof(rxMsg)) < 0){
        throw std::runtime_error("RX setup error: " + std::to_string(errno));
    }
     */
}

void ManifoldCAN::threadUpdate() {
    //TODO finish this
    canfd_frame frame;

    ssize_t size = read(canSocket, &frame, CANFD_MTU);

    FloatCANMessage message = receiveFloatMessage();
    float data = message.data;

    std::cout << "ID: " << std::hex << frame.can_id << " Data: " << data << " Message ID: " << message.id << std::endl;
}

void ManifoldCAN::writeTest() {
    for(float i = -1.0; i <= 1.0; i += .01){
        sendFloatMessage(FloatCANMessage(1, i));
        std::this_thread::sleep_for(1s);
    }
}