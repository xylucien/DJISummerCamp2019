#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <memory>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

using namespace std::chrono_literals;

#include "ManifoldCAN.h"

int main(int argc, char **argv) {
    std::cout << "Hello, World!" << std::endl;

    ros::init(argc, argv, "CANRobot");
    ros::NodeHandle n;

    ManifoldCAN can("can0");

    n.subscribe("robot/velocity", 1000, &ManifoldCAN::sendTargetVelocityROS, &can);

    ros::spin();

    return 0;
}