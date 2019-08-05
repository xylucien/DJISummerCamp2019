#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <memory>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include "ManifoldCAN.h"

void subTest(const geometry_msgs::Twist &twist){
    std::cout << "hello" << std::endl;
}

int main(int argc, char **argv) {
    std::cout << "Hello, World!" << std::endl;

    ros::init(argc, argv, "CANRobot");
    ros::NodeHandle n;

    ManifoldCAN can("can0", 1000.0);

    Twist2D zeroTwist;
    zeroTwist.vX = 0.0;
    zeroTwist.vY = 0.0;
    zeroTwist.w = 0.0;

    can.sendTargetVelocity(zeroTwist);

    //n.subscribe("cmd_vel", 1000, &ManifoldCAN::sendTargetVelocityROS, &can);
    ros::Subscriber sub = n.subscribe("cmd_vel", 1000, &ManifoldCAN::sendTargetVelocityROS, &can);
    ros::Subscriber servoSub = n.subscribe("servo_vel", 1000, &ManifoldCAN::sendTargetServoVelocity, &can);

    ros::Subscriber buzzerFreqSub = n.subscribe("buzzer/frequency", 1000, &ManifoldCAN::sendBuzzerFrequency, &can);
    ros::Subscriber buzzerDutyCycleSub = n.subscribe("buzzer/dutycycle", 1000, &ManifoldCAN::sendBuzzerDutyCycle, &can);

    std::shared_ptr<ros::Publisher> rollPub = std::make_shared<ros::Publisher>(n.advertise<std_msgs::Float32>("imu/roll", 1000));
    ros::Publisher pitchPub = n.advertise<std_msgs::Float32>("imu/pitch", 1000);
    ros::Publisher yawPub = n.advertise<std_msgs::Float32>("imu/yaw", 1000);

    ros::Rate loopRate(100);
    can.initialize(loopRate);

    can.addRosPublisher(ManifoldCAN::newCanId(0x600, CANMESSAGE_ID_AHRS, CANMESSAGE_SUBID_AHRS_ROLL), rollPub);
    can.addRosPublisher(ManifoldCAN::newCanId(0x600, CANMESSAGE_ID_AHRS, CANMESSAGE_SUBID_AHRS_PITCH), std::make_shared<ros::Publisher>(pitchPub));
    can.addRosPublisher(ManifoldCAN::newCanId(0x600, CANMESSAGE_ID_AHRS, CANMESSAGE_SUBID_AHRS_YAW), std::make_shared<ros::Publisher>(yawPub));

    ros::spin();

    //can.sendTargetVelocity(test);

    return 0;
}