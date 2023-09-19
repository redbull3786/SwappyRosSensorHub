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
 *   Arguments:
 *   - NodeName=<Name>
 *   - PublisherNode=<Name_Of_Published_Element>
 *   - SpiDevice=<SpiDevice>
 *  
 ***********************************************************/

#include "ros/ros.h"
#include "swappy_ros_sensor_hub/SensorHubState.h"
#include "protocol/Handler.hpp"
#include "protocol/AllSensorData.hpp"
#include "protocol/RadarActivity.hpp"
#include "configuration/NodeConfiguration.hpp"
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

NodeConfiguration parseArguments(int argc, char **argv)
{
   NodeConfiguration tmp;

   // parse elements:
   for (int i = 1; i < argc; i++)
   {
      Arguemnts argument = getKeyValue(std::string(argv[i]));

      if (argument.Key == std::string("NodeName"))
      {
         tmp.NodeName = argument.Value; //"SensorHubNodeFront";
      }
      else if (argument.Key == std::string("PublisherNode"))
      {
         tmp.PublisherName = argument.Value; //"frontSensors";
      }
      else if (argument.Key == std::string("SpiDevice"))
      {
         tmp.SpiConfig.SpiDevice = argument.Value; //"/dev/spi/spi1.0"
         tmp.SpiConfig.SpiSpeed_Hz = 17000000;
         tmp.SpiConfig.CsChange = 1;
         tmp.SpiConfig.BitsPerWord = 8;
      }
   }

   return tmp;   
}

int main(int argc, char **argv)
{
   // This needs to happen before we start fooling around with logger levels.  Otherwise the level we set may be overwritten by
   // a configuration file
   ROSCONSOLE_AUTOINIT;

   NodeConfiguration config = parseArguments(argc, argv);

   ROS_INFO("Start Sensor Publisher: %s", config.NodeName.c_str());

   ros::init(argc, argv, config.NodeName);
   ros::NodeHandle n;

   ros::Publisher node_pub = n.advertise<swappy_ros_sensor_hub::SensorHubState>(config.PublisherName, 1000);
   ros::Rate loop_rate(1); // 1Hz == 1 cycle per second

   Handler protocol_handler(config.SpiConfig);

   while (ros::ok())
   {
      swappy_ros_sensor_hub::SensorHubState msg;
      uint32_t messageCount = 0;

      AllSensorData allSensorData = protocol_handler.getAllSensorData();
      
      msg.UltraSonicSensorLeft_cm = allSensorData.getSensorData(Device::UltraSonicSenorLeft).getValue();
      msg.UltraSonicSensorMiddle_cm = allSensorData.getSensorData(Device::UltraSonicSenorMiddle).getValue();
      msg.UltraSonicSensorRight_cm = allSensorData.getSensorData(Device::UltraSonicSenorRight).getValue();
      msg.RadarSensorAktivity = allSensorData.getSensorData(Device::RadarSensor).getValue();
      msg.Temperature_celsius = 35.0;
   
      node_pub.publish(msg);

      // Debug Log:
      ROS_DEBUG("publish Message [Count: %d]: {Left[cm]: %f Middle[cm]: %f Right[cm]: %f Activity[Detect]: %d Temperature[Celcius]: %f}", 
            messageCount, msg.UltraSonicSensorLeft_cm, msg.UltraSonicSensorMiddle_cm, msg.UltraSonicSensorRight_cm,
            msg.RadarSensorAktivity, msg.Temperature_celsius);
      messageCount++;

      ros::spinOnce();

      loop_rate.sleep();
   }

   return 0;
}