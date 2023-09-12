/***********************************************************
 * 
 * Definition of Handler.hpp
 * 
 * Date: 11.09.2023
 * Version: 1.0
 * Creator: Sebastian Ehnert
 *  
 ***********************************************************/

#include "Device.hpp"
#include <inttypes.h>

class Handler
{
	Handler() = default;
	~Handler() = default;

	getSensorData();

	getAllSensorData();

	calibrateDevice();
};
