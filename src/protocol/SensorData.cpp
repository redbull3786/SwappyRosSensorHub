/***********************************************************
 * 
 * Implementation of SensorData.hpp
 * 
 * Date: 11.09.2023
 * Version: 1.0
 * Creator: Sebastian Ehnert
 *  
 ***********************************************************/

#include "SensorData.hpp"

SensorData::SensorData(const Device device, const float value)
: m_Device(device),
  m_Value(value)
{
}

Device SensorData::getDevice()
{
    return m_Device;
}

void SensorData::setDevice(const Device device)
{
    m_Device = device;
}

float SensorData::getValue()
{
    return m_Value;
}

void SensorData::setValue(const float value)
{
    m_Value = value;
}