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
#include "CANMessage.h"

using namespace std::chrono_literals;

ManifoldCAN::ManifoldCAN(const std::string &canInterface) {
    this->canInterfaceName = canInterface;

    canTxSocket = socket(PF_CAN, SOCK_DGRAM, CAN_BCM);

    if(canTxSocket < 0){
        throw std::runtime_error("Unable to start CAN TX socket errno: " + std::to_string(errno));
    }

    struct sockaddr_can addr;
    struct ifreq ifr;

    strcpy(ifr.ifr_name, canInterface.c_str());
    ioctl(canTxSocket, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    int connRet = connect(canTxSocket, (struct sockaddr *)&addr, sizeof(addr));

    if(connRet < 0){
        throw std::runtime_error("Connect failed with: " + std::to_string(errno));
    }

    canRxSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(canRxSocket < 0){
        throw std::runtime_error("Unable to start CAN RX socket errno: " + std::to_string(errno));
    }

    sendRXSetup();
}

int ManifoldCAN::sendFloatMessage(const FloatCANMessage &message) {
    BCM_Msg msg;
    msg.msg_head.opcode  = TX_SETUP;
    msg.msg_head.can_id  = calculateId(CANMESSAGE_MANIFOLD_BASE_ID, message.id, message.subid);
    msg.msg_head.flags   = SETTIMER|STARTTIMER|TX_CP_CAN_ID;
    msg.msg_head.nframes = 1;
    msg.msg_head.count = 0;
    msg.msg_head.ival1.tv_sec = 0;
    msg.msg_head.ival1.tv_usec = 0;
    msg.msg_head.ival2.tv_sec = 0;
    msg.msg_head.ival2.tv_usec = 100000;
    msg.frame[0].can_dlc   = 8;
    //std::cout << calculateId(CANMESSAGE_MANIFOLD_BASE_ID, message.id, message.subid) << std::endl;

    serializeFloat(message.data, msg.frame[0].data);

    return write(canTxSocket, &msg, sizeof(struct BCM_Msg));
}

void ManifoldCAN::sendRXSetup() const {
    BCM_Msg msg;
    msg.msg_head.opcode  = RX_SETUP;
    msg.msg_head.can_id  = 1639;
    msg.msg_head.flags   = 0;
    msg.msg_head.nframes = 1;

    write(canTxSocket, &msg, sizeof(struct BCM_Msg));
}

FloatCANMessage ManifoldCAN::receiveFloatMessage() const {
    sendRXSetup();

    BCM_Msg msg;

    ssize_t size = read(canTxSocket, &msg, sizeof(BCM_Msg));

    if(size <= 0){
        throw std::runtime_error("CAN returned zero bytes!");
    }

    FloatCANMessage message;

    message.id = deserializeInt(msg.frame[0].data);
    message.data = deserializeFloat(msg.frame[0].data + 4);

    return message;
}

void ManifoldCAN::sendTargetVelocityROS(const geometry_msgs::Twist &twist){
    Twist2D output;

    output.vX = twist.linear.x;
    output.vY = twist.linear.y;
    output.w = twist.angular.z;

    sendTargetVelocity(output);
}

void ManifoldCAN::sendTargetServoVelocity(const std_msgs::Float32 &msg) {
    FloatCANMessage message;
    message.id = CANMESSAGE_ID_PWM;
    message.subid = CANMESSAGE_SUBID_PWM0;

    message.data = msg.data;

    sendFloatMessage(message);
}

void ManifoldCAN::sendTargetVelocity(const Twist2D &twist) {
    sendFloatMessage(FloatCANMessage(CANMESSAGE_ID_TARGET_VEL, CANMESSAGE_SUBID_TARGET_VX, twist.vX));
    sendFloatMessage(FloatCANMessage(CANMESSAGE_ID_TARGET_VEL, CANMESSAGE_SUBID_TARGET_VY, twist.vY));
    sendFloatMessage(FloatCANMessage(CANMESSAGE_ID_TARGET_VEL, CANMESSAGE_SUBID_TARGET_VW, twist.w));

    sendFloatMessage(FloatCANMessage(CANMESSAGE_ID_TARGET_VEL, CANMESSAGE_SUBID_TARGET_READY, 1.0f));
}

void ManifoldCAN::sendBuzzerFrequency(const std_msgs::Float32 &msg) {
    FloatCANMessage message;
    message.id = CANMESSAGE_ID_BUZZER;
    message.subid = CANMESSAGE_SUBID_BUZZER_FREQUENCY;

    message.data = msg.data;

    sendFloatMessage(message);
}

void ManifoldCAN::sendBuzzerDutyCycle(const std_msgs::Float32 &msg) {
    FloatCANMessage message;
    message.id = CANMESSAGE_ID_BUZZER;
    message.subid = CANMESSAGE_SUBID_BUZZER_DUTYCYCLE;

    message.data = msg.data;

    sendFloatMessage(message);
}

//TODO RX
void ManifoldCAN::threadUpdate() {
    //TODO finish this
    canfd_frame frame;

    FloatCANMessage message = receiveFloatMessage();
    float data = message.data;

    std::cout << " Data: " << data << " Message ID: " << std::to_string(message.id) << std::endl;
}

void ManifoldCAN::writeTest() {
    //for(float i = -1.0; i <= 1.0; i += .01){
    //    sendFloatMessage(FloatCANMessage(1, i));
    //    std::this_thread::sleep_for(1s);
    //}
}

uint32_t ManifoldCAN::calculateId(uint8_t baseId, uint8_t canId, uint8_t subId) {
    uint32_t output = CANMESSAGE_MANIFOLD_BASE_ID;
    output |= (canId << 4);
    output |= subId;

    return output;
}