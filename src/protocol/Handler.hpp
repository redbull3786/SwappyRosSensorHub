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
#include <string>
#include "Device.hpp"
#include "SensorData.hpp"
#include "AllSensorData.hpp"
#include "ReturnState.hpp"
#include "../configuration/SpiConfiguration.hpp"

class Handler
{
public:
	Handler(const SpiConfiguration& spiConfig);
	~Handler();

	SensorData getSensorData();

	AllSensorData getAllSensorData();

	void calibrateDevice();

private:
	static const uint8_t S_MAX_BUFFER_SIZE = 255;

    SpiConfiguration m_SpiConfiguration;
	int m_FileDescriptor;
	uint8_t m_SendBuffer[S_MAX_BUFFER_SIZE];
	uint8_t m_ReceiveBuffer[S_MAX_BUFFER_SIZE];

    // SPI:
	ReturnState transfer(uint8_t* sendBuffer, uint8_t* receivedBuffer, uint16_t bufferSize);
	ReturnState init();
	ReturnState deinit();

    // Commands:
	void createGetSensorDataRequest(const Device device);
	SensorData parseGetSendorDataResponse();
	void createGetAllSensorDataRequest();
	AllSensorData parseGetAllSensorDataResponse();
	void createCalibrateDeviceRequest(const Device device);
	void parseCalibrateDeviceResponse(); 
};
