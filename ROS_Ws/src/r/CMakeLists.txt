cmake_minimum_required(VERSION 2.8.3)
project(r)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  nav_msgs
  tf
  apriltag
)


catkin_package()


include_directories( include ${catkin_INCLUDE_DIRS})


add_executable(r r.cpp)
target_link_libraries(r ${catkin_LIBRARIES})

