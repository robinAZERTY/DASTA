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
#include "pid.hpp"


class Dasta
{
    public :
// private:
    void configCommunication();
    void configSensorPreProcessing();
    void configActuators();
    void configureStateEstimate();
    Pid pidRx, pidRy, pidRz;
    Pid pidRoll, pidPitch, pidYaw;
    // Vector Wu = Vector(3); // angular velocity command
    float thrust = 0.0;
    bool inited = false;

public:
    Dasta();
    ~Dasta(){};
    void init();
    const bool getInited(){return inited;}
    void run_angular_velocity_control(float time=millis()/1000.0);
    void run_attitude_control(float time=millis()/1000.0);
    bool attitude_control_running = false;
    bool angular_velocity_control_running = false;
    Vector angular_velocity_command = Vector(3);
    void runDecisionOnUserEvent();
    SensorPreProcessing sensors;
    Communication communication;
    Actuators actuators;
    DecisionnalUnit decisionnal_unit;
    StateEstimate estimator;
};

#endif // DASTA_HPP