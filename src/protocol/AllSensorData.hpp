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
#include "Device.hpp"
#include "SensorData.hpp"

class AllSensorData
{
public:
	AllSensorData() = default;
	AllSensorData(const SensorData& sensor1, const SensorData& sensor2, const SensorData& sensor3, const SensorData& sensor4);
	~AllSensorData() = default;

	void setSensorData(const uint8_t index, const SensorData sensor);

	SensorData getSensorData(const uint8_t index);

private:
	SensorData m_Sensor1;
	SensorData m_Sensor2;
	SensorData m_Sensor3;
	SensorData m_Sensor4;
};