/*
cette classe regroupe toutes les fonctionnaités necessaires au fonctionnement du saint projet
*/

#ifndef DASTA_HPP
#define DASTA_HPP
#include "SensorPreProcessing.hpp"
#include "StateEstimate.hpp"
#include "Communication.hpp"
#include "Actuators.hpp"
#include "DecisionnalUnit.hpp"

class Dasta
{
    private:
     void configCommunication();
public:
    Dasta();
    ~Dasta(){};
    void runDecisionOnUserEvent();
    static SensorPreProcessing sensors;
    StateEstimate estimator;
    Communication communication; 
    static Actuators actuators;
    DecisionnalUnit decisionnal_unit;
};

#endif // DASTA_HPP