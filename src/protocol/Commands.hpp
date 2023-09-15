/***********************************************************
 * 
 * Definition of Commands.hpp
 * 
 * Date: 11.09.2023
 * Version: 1.0
 * Creator: Sebastian Ehnert
 *  
 ***********************************************************/

#ifndef PROTOCOL_COMMANDS_HPP
#define PROTOCOL_COMMANDS_HPP

#include <inttypes.h>

enum class Commands : uint8_t
{ 
    GetSensorData = 0x01,
    GetAllSensorData = 0x02,
    CalibrateDevice = 0x03
};

#endif // - PROTOCOL_COMMANDS_HPP