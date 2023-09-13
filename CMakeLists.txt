cmake_minimum_required(VERSION 2.8.3)
project(swappy_ros_sensor_hub)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    message_generation
)

## Declare ROS messages and services
add_message_files(DIRECTORY msg FILES 
    SensorHubState.msg
)

## Generate added messages and services
generate_messages()

## Declare a catkin package
catkin_package(
    CATKIN_DEPENDS message_runtime
)

## Build node and client
include_directories(include ${catkin_INCLUDE_DIRS})

## Find all Protocol SRC Files
file (GLOB SRC_FILES 
    src/protocol/*.hpp
    src/protocol/*.cpp 
)

add_executable(node src/node.cpp ${SRC_FILES})
target_link_libraries(node ${catkin_LIBRARIES})