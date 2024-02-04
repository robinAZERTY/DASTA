/*
cette classe regroupe toutes les fonctionnaités necessaires au fonctionnement du saint projet
*/

#ifndef DASTA_HPP
#define DASTA_HPP
#include "SensorPreProcessing.hpp"
#include "Communication.hpp"
#include "Actuators.hpp"
#include "DecisionnalUnit.hpp"
#include "StateEstimate.hpp"

class Dasta
{
private:
    void configCommunication();
    void configSensorPreProcessing();
    void configureStateEstimate();

public:
    Dasta(){};
    ~Dasta(){};
    void init();
    void runDecisionOnUserEvent();
    void runStateEstimate();
    SensorPreProcessing sensors;
    Communication communication;
    Actuators actuators;
    DecisionnalUnit decisionnal_unit;
    StateEstimate estimator;
};

#endif // DASTA_HPP