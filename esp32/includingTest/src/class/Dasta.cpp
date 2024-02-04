#include "Dasta.hpp"


void Dasta::init()
{
    while (!Serial)
        delay(10);
        
    configureStateEstimate();
    configSensorPreProcessing();
    configCommunication();
}