#include <iostream>
#include "ManifoldCAN.h"

int main() {
    std::cout << "Hello, World!" << std::endl;
    ManifoldCAN can("can0");

    for(;;){
        Twist2D test;
        test.vX = 3.14;
        test.vY = test.vX * 2;
        test.w = 1.0;
        can.sendTargetVelocity(test);
    }

    return 0;
}