/***********************************************************
 * 
 * Definition of NodeSpiConfiguration.hpp
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
   std::string NodeName;
   std::string PublisherName;
   SpiConfiguration SpiConfig;
};

#endif // - CONFIGURATION_NODECONFIGURATION_HPP