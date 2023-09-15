/***********************************************************
 * 
 * Implementation of Handler.hpp
 * 
 * Date: 11.09.2023
 * Version: 1.0
 * Creator: Sebastian Ehnert
 *  
 ***********************************************************/

#include "Handler.hpp"
#include "Commands.hpp"
#include <cstring>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <ros/rosco

Handler::Handler(const SpiConfiguration& spiConfig)
: m_SpiConfiguration(spiConfig),
  m_FileDescriptor(-1),
  m_SendBuffer(),
  m_ReceiveBuffer()
{
   init();
}

Handler::~Handler()
{
   deinit();
}

/********************
* Public Functions:
*********************/
SensorData Handler::getSensorData()
{
   return SensorData();
}

AllSensorData Handler::getAllSensorData()
{
   return AllSensorData();
}

void Handler::calibrateDevice()
{
}

/********************
* Private Functions:
*********************/

/********************
* SPI Functions:
*********************/
ReturnState Handler::transfer(uint8_t* sendBuffer, uint8_t* receivedBuffer, uint16_t bufferSize)
{
   return ReturnState::Success;
}

ReturnState Handler::init()
{
   ReturnState rc = ReturnState::Success;

   m_FileDescriptor = ::open(m_SpiConfiguration.SpiDevice.c_str(), O_RDWR);
   if(m_FileDescriptor < 0)
   {
      // Log with ROS console
   }

   return rc;
}

ReturnState Handler::deinit()
{
   ReturnState rc = ReturnState::Success;

   int statusValue = ::close(m_FileDescriptor);
   if (statusValue < 0)
   {
      // Log with ROS console
   }

   return rc;
}

/********************
* Command Functions:
*********************/
void Handler::createGetSensorDataRequest(const Device device)
{
   std::memset(m_SendBuffer, 0, S_MAX_BUFFER_SIZE);
   m_SendBuffer[0] = static_cast<uint8_t>(Commands::GetSensorData);
   m_SendBuffer[1] = static_cast<uint8_t>(device);
}

SensorData Handler::parseGetSendorDataResponse()
{
   SensorData tmp;
   ReturnState rc = static_cast<ReturnState>(m_ReceiveBuffer[0]);
   if(rc == ReturnState::Success)
   {
      tmp.setDevice(static_cast<Device>(m_ReceiveBuffer[1]));
      tmp.setValue(0.0);
   }

   return tmp;
}

void Handler::createGetAllSensorDataRequest()
{
   std::memset(m_SendBuffer, 0, S_MAX_BUFFER_SIZE);
   m_SendBuffer[0] = static_cast<uint8_t>(Commands::GetAllSensorData);
}

AllSensorData Handler::parseGetAllSensorDataResponse()
{
   AllSensorData tmp;
   return tmp;
}

void Handler::createCalibrateDeviceRequest(const Device device)
{
   std::memset(m_SendBuffer, 0, S_MAX_BUFFER_SIZE);
   m_SendBuffer[0] = static_cast<uint8_t>(Commands::CalibrateDevice);
   m_SendBuffer[1] = static_cast<uint8_t>(device);
}

void Handler::parseCalibrateDeviceResponse()
{

}
