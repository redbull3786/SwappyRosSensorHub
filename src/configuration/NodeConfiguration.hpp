/***********************************************************
 * 
 * Definition of NodeConfiguration.hpp
 * 
 * Date: 15.09.2023
 * Version: 1.0
 * Creator: Sebastian Ehnert
 *  
 ***********************************************************/

#ifndef CONFIGURATION_NODECONFIGURATION_HPP
#define CONFIGURATION_NODECONFIGURATION_HPP

#include <string>
#include "SpiConfiguration.hpp"

struct NodeConfiguration
{
   // Name of the node 
   std::string NodeName;

   // Name of published element
   std::string PublisherName;

   // Configuration of the SPI device
   SpiConfiguration SpiConfig;
};

#endif // - CONFIGURATION_NODECONFIGURATION_HPP