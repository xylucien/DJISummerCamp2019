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

ManifoldCAN::ManifoldCAN(const std::string &canInterface, double rate) : rxThreadRate(rate), rosPubThreadRate(rate) {
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

    memset(&addr, 0, sizeof(sockaddr_can));
    memset(&ifr, 0, sizeof(ifreq));

    std::strncpy(ifr.ifr_name, canInterface.c_str(), IFNAMSIZ);
    ioctl(canRxSocket, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    int rxSocketBind = bind(
            canRxSocket,
            reinterpret_cast<struct sockaddr*>(&addr),
            sizeof(addr)
    );

    if(rxSocketBind < 0){
        throw std::runtime_error("Unable to bind CAN RX socket errno: " + std::to_string(errno));
    }
}

void ManifoldCAN::initialize(const ros::Rate &rxUpdateRate) {
    if(!isRxThreadInitialized) {
        isRxThreadInitialized = true;
        rxThreadRate = rxUpdateRate;
        rxThread = std::make_shared<std::thread>(std::bind(&ManifoldCAN::rxThreadUpdate, this));
    }

    if(!isRosPubThreadInitialized){
        isRosPubThreadInitialized = true;
        rosPubThreadRate = rxUpdateRate;
        rosThread = std::make_shared<std::thread>(std::bind(&ManifoldCAN::rosPubThreadUpdate, this));
    }
}

void ManifoldCAN::addRosPublisher(const CANId &id, std::shared_ptr<ros::Publisher> publisher) {
    publisherListMutex.lock();
    publisherList[id.messageId].emplace(id.subId, publisher);
    publisherListMutex.unlock();
}

void ManifoldCAN::addRosPublisher(const CANId &id, ros::Publisher &publisher) {
    addRosPublisher(id, std::make_shared<ros::Publisher>(publisher));
}

CANId ManifoldCAN::newCanId(uint32_t baseId, uint8_t messageId, uint8_t subId) {
    CANId id;
    id.baseId = baseId;
    id.messageId = messageId;
    id.subId = subId;

    return(id);
}

int ManifoldCAN::sendFloatMessage(const FloatCANMessage &message) {
    BCM_Msg msg;
    msg.msg_head.opcode  = TX_SETUP;
    msg.msg_head.can_id  = serializeId(newCanId(0x600, message.id, message.subid));
    msg.msg_head.flags   = SETTIMER|STARTTIMER|TX_CP_CAN_ID;
    msg.msg_head.nframes = 1;
    msg.msg_head.count = 0;
    msg.msg_head.ival1.tv_sec = 0;
    msg.msg_head.ival1.tv_usec = 0;
    msg.msg_head.ival2.tv_sec = 0;
    msg.msg_head.ival2.tv_usec = 100000;
    msg.frame[0].can_dlc   = 8;

    serializeFloat(message.data, msg.frame[0].data);

    return write(canTxSocket, &msg, sizeof(struct BCM_Msg));
}

void ManifoldCAN::sendRXSetup() const {
    BCM_Msg msg;
    msg.msg_head.opcode  = RX_SETUP;
    msg.msg_head.can_id = 0x650;
    msg.msg_head.flags   = 0;
    msg.msg_head.nframes = 1;

    write(canTxSocket, &msg, sizeof(struct BCM_Msg));
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

void ManifoldCAN::rosPubThreadUpdate() {
    while(ros::ok()){
        if(!receivedCanMessages.empty()){
            rxMessageQueueMutex.lock();
            auto canMsg = receivedCanMessages.front();
            receivedCanMessages.pop();
            rxMessageQueueMutex.unlock();

            CANId id = canMsg.id;

            publisherListMutex.lock();
            auto subIdMap = publisherList.find(id.messageId);
            auto pub = subIdMap->second.find(id.subId);
            publisherListMutex.unlock();

            if(pub != subIdMap->second.end()){
                //Found element
                //Assume everything is float32 for now
                std_msgs::Float32 msg;
                msg.data = deserializeFloat((uint8_t*) canMsg.data);

                (pub->second)->publish(msg);
                //std::cout << (pub->second)->getTopic() << std::to_string(id.baseId >> 8) << ',' << std::to_string(id.messageId) << ',' << std::to_string(id.subId) << "found" << msg.data << std::endl;
            }

            free(canMsg.data);
        }

        //rosPubThreadRate.sleep();
    }
}

//TODO RX
void ManifoldCAN::rxThreadUpdate() {
    while(ros::ok()){
        //sendRXSetup();

        struct canfd_frame frame;

        read(canRxSocket, &frame, CANFD_MTU);

        CANMessage newMsg;
        newMsg.id = deserializeId(frame.can_id);

        newMsg.data = malloc(sizeof(frame.data));
        memcpy(newMsg.data, frame.data, sizeof(frame.data));

        rxMessageQueueMutex.lock();
        receivedCanMessages.push(newMsg);
        rxMessageQueueMutex.unlock();

        //rxThreadRate.sleep();
    }
}