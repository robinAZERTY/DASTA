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
    float Crx = pidRx.compute(communication.angular_velocity_command.data[0] - sensors.gyro.data[0], time);
    float Cry = pidRy.compute(communication.angular_velocity_command.data[1] - sensors.gyro.data[1], time);
    float Crz = pidRz.compute(communication.angular_velocity_command.data[2] - sensors.gyro.data[2], time);

    actuators.motor1.write(Crx+Cry-Crz+thrust);
    actuators.motor2.write(-Crx+Cry+Crz+thrust);
    actuators.motor3.write(-Crx-Cry-Crz+thrust);
    actuators.motor4.write(Crx-Cry+Crz+thrust);
}