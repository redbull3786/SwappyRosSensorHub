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

SensorData::SensorData(const Device device, const uint8_t value)
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

uint8_t SensorData::getValue()
{
    return m_Value;
}

void SensorData::setValue(const uint8_t value)
{
    m_Value = value;
}