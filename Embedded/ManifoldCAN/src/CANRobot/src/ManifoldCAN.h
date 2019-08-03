//
// Created by Abiel on 7/31/19.
//

#ifndef MANIFOLDCAN_MANIFOLDCAN_H
#define MANIFOLDCAN_MANIFOLDCAN_H

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/bcm.h>
#include <sys/ioctl.h>

#include <string>
#include <libnet.h>
#include <mutex>
#include <fcntl.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include "MathUtils.h"
#include "CANMessage.h"
#include "CANTopic.h"

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

struct FloatCANMessage {
    FloatCANMessage(){
        this->id = 0;
        this->subid = 0;
        this->data = 0;
    }

    FloatCANMessage(uint8_t id, uint8_t subid, float data){
        this->id = id;
        this->subid = subid;

        this->data = data;
    }
    uint8_t id;
    uint8_t subid;

    float data;
};

struct BCM_Msg {
    struct bcm_msg_head msg_head;
    struct can_frame frame[4]; /* just an example */
} ;


class ManifoldCAN {
public:
    ManifoldCAN(const std::string &canInterface);

    void sendTargetVelocity(const Twist2D &twist);
    void sendTargetVelocityROS(const geometry_msgs::Twist &twist);


private:
    static uint32_t calculateId(uint8_t baseId, uint8_t canId, uint8_t subId);

    void threadUpdate();
    void writeTest();

    int sendFloatMessage(const FloatCANMessage &message);
    FloatCANMessage receiveFloatMessage() const;

    void sendRXSetup() const;

    std::string canInterfaceName;
    int canTxSocket;
    int canRxSocket;

    struct sockaddr_can addr;
    struct ifreq ifr;
};


#endif //MANIFOLDCAN_MANIFOLDCAN_H
