/***********************************************************
 * 
 * Definition of AllSensorData.hpp
 * 
 * Date: 11.09.2023
 * Version: 1.0
 * Creator: Sebastian Ehnert
 *  
 ***********************************************************/

#include <inttypes.h>
#include <map>
#include "Device.hpp"
#include "SensorData.hpp"

class AllSensorData
{
public:
	AllSensorData() = default;
	AllSensorData(const SensorData& sensor1, 
			 	  const SensorData& sensor2, 
				  const SensorData& sensor3, 
				  const SensorData& sensor4,
				  const SensorData& sensor5);
	~AllSensorData() = default;

	void setSensorData(const Device device, const SensorData& data);

	SensorData getSensorData(const Device device);

private:
	std::map<Device, SensorData> m_SensorData;
};