cmake_minimum_required(VERSION 3.5.1)
project(ManifoldCAN)

set(CMAKE_CXX_STANDARD 14)

include_directories(src)
add_executable(ManifoldCAN src/main.cpp src/ManifoldCAN.cpp src/CANMessage.c src/CANUtil.c)