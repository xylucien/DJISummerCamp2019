//
// Created by Abiel on 7/31/19.
//

#ifndef MANIFOLDCAN_MANIFOLDCAN_H
#define MANIFOLDCAN_MANIFOLDCAN_H

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/bcm.h>
#include <sys/ioctl.h>

#include <thread>
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
#include "CANUtil.h"

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

struct FloatCANMessage {
    FloatCANMessage() {
        this->id = 0;
        this->subid = 0;
        this->data = 0;
    }

    FloatCANMessage(uint8_t id, uint8_t subid, float data) {
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
    struct can_frame frame[1];
};

class ManifoldCAN {
public:
    ManifoldCAN(const std::string &canInterface, double rate, tf::TransformBroadcaster &transformBroadcaster, std::shared_ptr<ros::Publisher> odomPublish);

    void initialize(const ros::Rate &rxUpdateRate);

    void addRosPublisher(const CANId &id, std::shared_ptr <ros::Publisher> publisher);
    void addRosPublisher(const CANId &id, ros::Publisher& publisher);

    static CANId newCanId(uint32_t baseId, uint8_t messageId, uint8_t subId);

    void sendTargetVelocity(const Twist2D &twist);

    void sendTargetVelocityROS(const geometry_msgs::Twist &twist);

    void sendTargetServoVelocity(const std_msgs::Float32 &msg);

    void sendBuzzerFrequency(const std_msgs::Float32 &msg);

    void sendBuzzerDutyCycle(const std_msgs::Float32 &msg);

    void sendRightBall(const std_msgs::Float32 &msg);
    void sendCenterBall(const std_msgs::Float32 &msg);
    void sendLeftBall(const std_msgs::Float32 &msg);

private:
    int sendFloatMessage(const FloatCANMessage &message);

    void sendRXSetup() const;

    void rxThreadUpdate();

    bool isRxThreadInitialized = false;
    std::shared_ptr <std::thread> rxThread;
    ros::Rate rxThreadRate;

    void rosPubThreadUpdate();
    void updateOdom();

    bool isRosPubThreadInitialized = false;
    std::shared_ptr <std::thread> rosThread;
    ros::Rate rosPubThreadRate;

    std::queue <CANMessage> receivedCanMessages;
    std::mutex rxMessageQueueMutex;

    std::map <uint8_t, std::map<uint8_t, std::shared_ptr<ros::Publisher>>> publisherList;
    std::mutex publisherListMutex;

    std::shared_ptr<ros::Publisher> odomPublish;
    geometry_msgs::TransformStamped tfOdom;
    nav_msgs::Odometry odom;
    bool updatedX = false;
    bool updatedY = false;
    bool updatedW = false;
    float yaw = 0;
    std::mutex odomMutex;

    tf::TransformBroadcaster transformBroadcaster;

    std::string canInterfaceName;
    int canTxSocket;
    int canRxSocket;

    struct sockaddr_can addr;
    struct ifreq ifr;
};

#endif //MANIFOLDCAN_MANIFOLDCAN_H
