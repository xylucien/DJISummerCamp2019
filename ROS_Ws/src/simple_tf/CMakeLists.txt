cmake_minimum_required(VERSION 3.1)
project(simple_tf)

set(CMAKE_CXX_STANDARD 11)

#############################
#Dependencies
#############################

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    geometry_msgs
)
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(tf_test tf_test.cpp)
target_link_libraries(tf_test ${catkin_LIBRARIES})