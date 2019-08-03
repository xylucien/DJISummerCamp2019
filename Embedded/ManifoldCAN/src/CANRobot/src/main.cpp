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

    ManifoldCAN can("can0");

    Twist2D zeroTwist;
    zeroTwist.vX = 0.0;
    zeroTwist.vY = 0.0;
    zeroTwist.w = 0.0;

    can.sendTargetVelocity(zeroTwist);

    n.subscribe("cmd_vel", 1000, &ManifoldCAN::sendTargetVelocityROS, &can);
    ros::Subscriber sub = n.subscribe("cmd_vel", 1000, &ManifoldCAN::sendTargetVelocityROS, &can);


    //can.sendTargetVelocity(test);

    ros::spin();

    return 0;
}