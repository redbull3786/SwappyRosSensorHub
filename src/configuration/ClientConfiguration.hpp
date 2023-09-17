/***********************************************************
 * 
 * Definition of ClientConfiguration.hpp
 * 
 * Date: 17.09.2023
 * Version: 1.0
 * Creator: Sebastian Ehnert
 *  
 ***********************************************************/

#ifndef CONFIGURATION_CLIENTCONFIGURATION_HPP
#define CONFIGURATION_CLIENTCONFIGURATION_HPP

#include <string>

struct ClientConfiguration
{
   // Name of the client node 
   std::string NodeName;

   // Name of the subscribed element (must be the same as like as PublisherName)
   std::string SubscriberName;
};

#endif // - CONFIGURATION_CLIENTCONFIGURATION_HPP