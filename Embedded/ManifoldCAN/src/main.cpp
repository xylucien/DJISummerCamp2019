#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

#include "ManifoldCAN.h"

int main() {
    std::cout << "Hello, World!" << std::endl;
    ManifoldCAN can("can0");

    for(;;){
        Twist2D test;
        for(float i = 0.0; i <= 1.0; i += 0.1) {
            test.vX = i + 1.0;
            test.vY = i + 2.0;
            test.w = std::sin(i);
            can.sendTargetVelocity(test);
            std::this_thread::sleep_for(100ms);
            can.threadUpdate();
            std::this_thread::sleep_for(100ms);
        }
    }

    return 0;
}