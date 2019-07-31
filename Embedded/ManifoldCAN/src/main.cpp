#include <iostream>
#include "ManifoldCAN.h"

int main() {
    std::cout << "Hello, World!" << std::endl;
    ManifoldCAN can("can0");

    for(;;){
        can.writeTest();
    }

    return 0;
}