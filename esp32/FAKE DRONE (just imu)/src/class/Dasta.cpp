#include "Dasta.hpp"


Dasta::Dasta()
{
    // configureStateEstimate();
    configSensorPreProcessing();
    configCommunication();
    configActuators();
}

void Dasta::run_anguler_velocity_control(float time)
{
    // read the sensors
    sensors.readSensors();

    // comput pid and write the control signal
    actuators.motor1.write(pidRx.compute(Wu.data[0] - sensors.gyro.data[0], time));
    actuators.motor2.write(pidRy.compute(Wu.data[1] - sensors.gyro.data[1], time));
    actuators.motor3.write(pidRz.compute(Wu.data[2] - sensors.gyro.data[2], time));
    actuators.motor4.write(pidRz.compute(Wu.data[2] - sensors.gyro.data[2], time));
}