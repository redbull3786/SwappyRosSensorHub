/***********************************************************
 * 
 * Definition of Handler.hpp
 * 
 * Date: 11.09.2023
 * Version: 1.0
 * Creator: Sebastian Ehnert
 *  
 ***********************************************************/

#include <inttypes.h>
#include "Device.hpp"
#include "SensorData.hpp"
#include "AllSensorData.hpp"

class Handler
{
public:
	Handler() = default;
	~Handler() = default;

	SensorData getSensorData();

	AllSensorData getAllSensorData();

	void calibrateDevice();
};
