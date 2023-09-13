/***********************************************************
 * 
 * Definition of Device.hpp
 * 
 * Date: 11.09.2023
 * Version: 1.0
 * Creator: Sebastian Ehnert
 *  
 ***********************************************************/

#ifndef PROTOCOL_DEVICE_HPP
#define PROTOCOL_DEVICE_HPP

#include <inttypes.h>

enum class Device : uint8_t
{ 
    UltraSonicSenorLeft = 0x00,
    UltraSonicSenorMiddle = 0x01,
    UltraSonicSenorRight = 0x02,
    RadarSensor = 0x03,
    TemperatureSensor = 0x04,
    All = 0x05
};

#endif // - PROTOCOL_DEVICE_HPP
