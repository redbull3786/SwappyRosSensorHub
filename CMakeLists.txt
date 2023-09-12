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
catkin_package()

