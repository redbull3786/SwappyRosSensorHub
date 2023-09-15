/***********************************************************
 * 
 * Definition of SensorData.hpp
 * 
 * Date: 11.09.2023
 * Version: 1.0
 * Creator: Sebastian Ehnert
 *  
 ***********************************************************/

#ifndef PROTOCOL_SENSOR_DATA_HPP
#define PROTOCOL_SENSOR_DATA_HPP

#include <inttypes.h>
#include "Device.hpp"

class SensorData
{
public:
	SensorData() = default;
	SensorData(const Device device, const float value);
	~SensorData() = default;

	Device getDevice();
	void setDevice(const Device device);

	float getValue();
	void setValue(const float value);

private:
	Device m_Device;
	float m_Value;
};

#endif // - PROTOCOL_SENSOR_DATA_HPP