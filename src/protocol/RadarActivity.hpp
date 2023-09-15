/***********************************************************
 * 
 * Definition of RadarActivity.hpp
 * 
 * Date: 11.09.2023
 * Version: 1.0
 * Creator: Sebastian Ehnert
 *  
 ***********************************************************/

#ifndef PROTOCOL_RADARACTIVITY_HPP
#define PROTOCOL_RADARACTIVITY_HPP

#include <inttypes.h>

enum class RadarActivity : uint8_t
{ 
    ActivityNotDetected = 0x00,
    ActivityDetected = 0x01,
};

#endif // - PROTOCOL_RADARACTIVITY_HPP