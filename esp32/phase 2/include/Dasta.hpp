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

enum class ControlMode
{
    ANGULAR_VELOCITY = 0,
    ATTITUDE,
    VELOCITY,
    POSITION
};

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
    ControlMode control_mode = ControlMode::ATTITUDE;
    int64_t now = 0;
    Dasta();
    ~Dasta(){};
    void init();
    const bool getInited(){return inited;}
    void run(const float dt);
    void run_angular_velocity_control(float dt);
    void run_attitude_control(float dt);
    bool attitude_control_running = false;
    bool angular_velocity_control_running = false;
    Vector angular_velocity_command = Vector(3);
    Vector rpy_offset = Vector(3);
    void set_rpy_offset();
    void runDecisionOnUserEvent();
    SensorPreProcessing sensors;
    Communication communication;
    Actuators actuators;
    DecisionnalUnit decisionnal_unit;
    StateEstimate estimator;
};

#endif // DASTA_HPP