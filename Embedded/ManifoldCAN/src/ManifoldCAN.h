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

struct FloatCANMessage {
    FloatCANMessage(){
        this->id = 0;
        this->data = 0;
    }

    FloatCANMessage(uint8_t id, float data){
        this->id = id;
        this->data = data;
    }
    uint8_t id;
    float data;
};

struct BCM_Msg {
    struct bcm_msg_head msg_head;
    struct can_frame frame[4]; /* just an example */
} ;


class ManifoldCAN {
public:
    ManifoldCAN(const std::string &canInterface);

    void threadUpdate();
    void writeTest();

    void sendTargetVelocity(const Twist2D &twist);


private:
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
