project(simple_msgs)
cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
find_package(catkin REQUIRED COMPONENTS
    std_msgs
    geometry_msgs
    message_generation
    )

add_message_files(
    DIRECTORY msg
    FILES
    RegionOccupy.msg
    CastleEnergy.msg
    GameStatus.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package()