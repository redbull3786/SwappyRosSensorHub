/***********************************************************
 * 
 * Implementation of node.cpp
 * 
 * Date: 11.09.2023
 * Version: 1.0
 * Creator: Sebastian Ehnert
 * 
 * Description:
 *   Implements a publisher to push all Sensor Data to ROS Master
 *  
 ***********************************************************/

#include "ros/ros.h"
#include "swappy_ros_sensor_hub/SensorHubState.h"
#include "protocol/Handler.hpp"
#include "protocol/AllSensorData.hpp"

int main(int argc, char **argv)
{
   ros::init(argc, argv, "SensorHubNode");
   ros::NodeHandle n;

   ros::Publisher node_pub = n.advertise<swappy_ros_sensor_hub::SensorHubState>("node", 1000);
   ros::Rate loop_rate(1); // 1Hz == 1 cycle per second

   Handler protocol_handler;

   while (ros::ok())
   {
      swappy_ros_sensor_hub::SensorHubState msg;

      AllSensorData allSensorData = protocol_handler.getAllSensorData();
      
      msg.UltraSonicSensorLeft_cm = allSensorData.getSensorData(Device::UltraSonicSenorLeft).getValue();
      msg.UltraSonicSensorMiddle_cm = allSensorData.getSensorData(Device::UltraSonicSenorMiddle).getValue();
      msg.UltraSonicSensorRight_cm = allSensorData.getSensorData(Device::UltraSonicSenorRight).getValue();
   
      node_pub.publish(msg);

      ros::spinOnce();

      loop_rate.sleep();
   }

   return 0;
}