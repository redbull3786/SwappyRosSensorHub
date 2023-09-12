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
#include "SwappyRosSensorHub/SensorHubState.h"
#include "protocol/Handler.hpp"
#include "protocol/AllSensorData.hpp"

int main(int argc, char **argv)
{
   ros::init(argc, argv, "SensorHubNode");
   ros::NodeHandle n;

   ros::Publisher node_pub = n.advertise<SwappyRosSensorHub::SensorHubState>("node", 1000);
   ros::Rate loop_rate(1); // 1Hz == 1 cycle per second

   Handler protocol_handler;

   while (ros::ok())
   {
      SwappyRosSensorHub::SensorHubState msg;

      AllSensorData SensorData = protocol_handler.getAllSensorData();

      node_pub.publish(msg);

      ros::spinOnce();

      loop_rate.sleep();
   }

   return 0;
}