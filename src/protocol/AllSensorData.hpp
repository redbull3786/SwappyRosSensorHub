/***********************************************************
 * 
 * Definition of AllSensorData.hpp
 * 
 * Date: 11.09.2023
 * Version: 1.0
 * Creator: Sebastian Ehnert
 *  
 ***********************************************************/

#ifndef PROTOCOL_ALL_SENSOR_DATA_HPP
#define PROTOCOL_ALL_SENSOR_DATA_HPP

#include <inttypes.h>
#include <map>
#include "Device.hpp"
#include "SensorData.hpp"

class AllSensorData
{
public:
	AllSensorData() = default;
	AllSensorData(SensorData& sensor1, 
			 	  SensorData& sensor2, 
				  SensorData& sensor3, 
				  SensorData& sensor4,
			      SensorData& sensor5);
	~AllSensorData() = default;

	void setSensorData(const Device device, const SensorData& data);

	SensorData getSensorData(const Device device);

private:
	std::map<Device, SensorData> m_SensorData;
};

#endif // - PROTOCOL_ALL_SENSOR_DATA_HPP