/*
cette classe regroupe toutes les fonctionnaités necessaires au fonctionnement du saint projet
*/

#ifndef DASTA_HPP
#define DASTA_HPP
#include "SensorPreProcessing.hpp"
#include "Communication.hpp"
#include "Actuators.hpp"
#include "DecisionnalUnit.hpp"
// #include "StateEstimate.hpp"
#include "pid.hpp"


class Dasta
{
    public :
// private:
    void configCommunication();
    void configSensorPreProcessing();
    void configActuators();
    Pid pidRx, pidRy, pidRz;
    // Vector Wu = Vector(3); // angular velocity command
    float thrust = 0.0;
    // void configureStateEstimate();

public:
    Dasta();
    ~Dasta(){};
    void run_anguler_velocity_control(float time=millis());
    void runDecisionOnUserEvent();
    SensorPreProcessing sensors;
    Communication communication;
    Actuators actuators;
    DecisionnalUnit decisionnal_unit;
    // StateEstimate estimator;
};

#endif // DASTA_HPP