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

AllSensorData::AllSensorData(const SensorData& sensor1, 
                             const SensorData& sensor2, 
                             const SensorData& sensor3, 
                             const SensorData& sensor4,
                             const SensorData& sensor5)
: m_SensorData()
{
   m_SensorData[sensor1.getDevice()] = sensor1;
   m_SensorData[sensor2.getDevice()] = sensor2;
   m_SensorData[sensor3.getDevice()] = sensor3;
   m_SensorData[sensor4.getDevice()] = sensor4;
   m_SensorData[sensor5.getDevice()] = sensor5;
}

void AllSensorData::setSensorData(const Device device, const SensorData& data)
{
   m_SensorData[device] = data;
}

SensorData AllSensorData::getSensorData(const Device device)
{
   return m_SensorData.find(device)->second;
}