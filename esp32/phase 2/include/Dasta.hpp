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
    float thrust = 0.0;
    bool inited = false;
    Vector angular_velocity_command_der = Vector(3);
    Vector last_angular_velocity_command = Vector(3);
    data_type remote_angular_velocity_sensitive = 1;
    data_type remote_attitude_sensitive = 0.1;
    data_type remote_yaw_sensitive = 1;
    void compute_angular_velocity_command_for_attitude_control(float dt);
    Vector kalman_delay = Vector(2);
    Vector close_loop_delay = Vector(2);
    Vector stream_delay = Vector(2);


public:
    ControlMode control_mode = ControlMode::ATTITUDE;
    int64_t now = 0;
    Dasta();
    ~Dasta(){};
    void init();
    const bool getInited(){return inited;}
    void control(const float dt);

    void run_angular_velocity_control(float dt);
    void run_attitude_control(float dt);
    Vector angular_velocity_command = Vector(3);
    Vector rpy_command = Vector(3);
    Vector remote_commands = Vector(4);
    
    
    void runDecisionOnUserEvent();


    SensorPreProcessing sensors;
    Communication communication;
    Actuators actuators;
    DecisionnalUnit decisionnal_unit;
    StateEstimate estimator;
};

#endif // DASTA_HPP