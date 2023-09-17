/***********************************************************
 * 
 * Implementation of client.cpp
 * 
 * Date: 17.09.2023
 * Version: 1.0
 * Creator: Sebastian Ehnert
 * 
 * Description:
 *   Implements a subscriber to get all Sensor Data from ROS Master
 * 
 *   Arguments:
 *   - NodeName=<Name>
 *   - SubscriberName=<Name_Of_Subcribed_Element>
 *  
 ***********************************************************/

#include "ros/ros.h"
#include "swappy_ros_sensor_hub/SensorHubState.h"
#include "configuration/ClientConfiguration.hpp"
#include <ros/console.h>
#include "Arguments.hpp"

Arguemnts getKeyValue(const std::string& arg)
{
   Arguemnts tmp;

   size_t pos = arg.find("=");
   tmp.Key = arg.substr(0, pos);
   tmp.Value = arg.substr((pos + 1), (arg.size() - (pos + 1)));

   return tmp;
}

ClientConfiguration parseArguments(int argc, char **argv)
{
   ClientConfiguration tmp;

   // parse elements:
   for (int i = 1; i < argc; i++)
   {
      Arguemnts argument = getKeyValue(std::string(argv[i]));

      if (argument.Key == "NodeName")
      {
         tmp.NodeName = argument.Value; //"SensorHubClientFront";
      }
      else if (argument.Key == "SubscriberName")
      {
         tmp.SubscriberName = argument.Value; //"frontSensors";
      }
   }

   return tmp;   
}

void sensorCallback(const swappy_ros_sensor_hub::SensorHubState::ConstPtr& msg)
{
  
}

int main(int argc, char **argv)
{
   ClientConfiguration config = parseArguments(argc, argv);

   ros::init(argc, argv, config.NodeName);

   ros::NodeHandle n;
   ros::Subscriber sub = n.subscribe(config.SubscriberName, 1000, sensorCallback);

   ros::spin();

   return 0;
}