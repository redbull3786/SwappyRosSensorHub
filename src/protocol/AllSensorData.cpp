/***********************************************************
 * 
 * Implementation of AllSensorData.cpp
 * 
 * Date: 11.09.2023
 * Version: 1.0
 * Creator: Sebastian Ehnert
 *  
 ***********************************************************/

#include "AllSensorData.hpp"

AllSensorData::AllSensorData(const SensorData& sensor1, const SensorData& sensor2, const SensorData& sensor3, const SensorData& sensor4)
: m_Sensor1(sensor1),
  m_Sensor2(sensor2),
  m_Sensor3(sensor3),
  m_Sensor4(sensor4)
{
}

void AllSensorData::setSensorData(const uint8_t index, const SensorData sensor)
{
   switch (index)
   {
      case 1: m_Sensor1 = sensor; break;
      case 2: m_Sensor2 = sensor; break;
      case 3: m_Sensor3 = sensor; break;
      case 4: m_Sensor4 = sensor; break;
      default:
         break;
   }
}

SensorData AllSensorData::getSensorData(const uint8_t index)
{
   SensorData tmp;
   switch (index)
   {
      case 1: tmp = m_Sensor1; break;
      case 2: tmp = m_Sensor2; break;
      case 3: tmp = m_Sensor3; break;
      case 4: tmp = m_Sensor4; break;
      default:
         break;
   }

   return tmp;
}