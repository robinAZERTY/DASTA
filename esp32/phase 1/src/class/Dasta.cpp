#include "Dasta.hpp"

Dasta::Dasta()
{
}
void Dasta::init()
{
    configureStateEstimate();
    configSensorPreProcessing();
    configCommunication();
    configActuators();
    communication.device_name = "ESP32-Bluetooth";
    communication.start();
    delay(1000); // wait for the serial monitor to open
    sensors.init();
    inited = true;
    
}
void Dasta::run_anguler_velocity_control(float time)
{
    // read the sensors
    // sensors.readSensors();

    // comput pid and write the control signal
    float Crx = pidRx.compute(communication.angular_velocity_command.data[0] - sensors.gyro.data[0], time);
    float Cry = pidRy.compute(communication.angular_velocity_command.data[1] - sensors.gyro.data[1], time);
    float Crz = pidRz.compute(communication.angular_velocity_command.data[2] - sensors.gyro.data[2], time);

    float c1 = Crx + Cry - Crz;
    float c2 = -Crx + Cry + Crz;
    float c3 = -Crx - Cry - Crz;
    float c4 = Crx - Cry + Crz;

    // float max = c1;
    // if (c2 > max)
    //     max = c2;
    // if (c3 > max)
    //     max = c3;
    // if (c4 > max)
    //     max = c4;

    // if (max + thrust > 1)
    // {
    //     Crx /= max + thrust;
    //     Cry /= max + thrust;
    //     Crz /= max + thrust;
    // }

    actuators.motor1.write(c1 + thrust);
    actuators.motor2.write(c2 + thrust);
    actuators.motor3.write(c3 + thrust);
    actuators.motor4.write(c4 + thrust);
}

void Dasta::run_attitude_control(float time)
{
    if (!attitude_control_running)
        return;

    // read the sensors
    // sensors.readSensors();

    // comput pid and write the control signal
    float Crx = pidRx.compute(communication.rpy_command.data[0] - estimator.rpy.data[0], time);
    float Cry = pidRy.compute(communication.rpy_command.data[1] - estimator.rpy.data[1], time);
    float Crz = pidRz.compute(communication.angular_velocity_command.data[2] - sensors.gyro.data[2], time);

    float c1 = Crx + Cry - Crz;
    float c2 = -Crx + Cry + Crz;
    float c3 = -Crx - Cry - Crz;
    float c4 = Crx - Cry + Crz;

    actuators.motor1.write(c1 + thrust);
    actuators.motor2.write(c2 + thrust);
    actuators.motor3.write(c3 + thrust);
    actuators.motor4.write(c4 + thrust);
}