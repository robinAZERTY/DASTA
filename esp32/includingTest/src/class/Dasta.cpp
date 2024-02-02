#include "Dasta.hpp"


Dasta::Dasta()
{
    configureStateEstimate();
    configSensorPreProcessing();
    configCommunication();
}