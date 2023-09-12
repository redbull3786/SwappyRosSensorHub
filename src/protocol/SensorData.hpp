/***********************************************************
 * 
 * Definition of SensorData.hpp
 * 
 * Date: 11.09.2023
 * Version: 1.0
 * Creator: Sebastian Ehnert
 *  
 ***********************************************************/

#include <inttypes.h>
#include "Device.hpp"

class SensorData
{
public:
	SensorData() = default;
	SensorData(const Device device, const uint8_t value);
	~SensorData() = default;

	Device getDevice();
	void setDevice(const Device device);

	uint8_t getValue();
	void setValue(const uint8_t value);

private:
	Device m_Device;
	uint8_t m_Value;
};