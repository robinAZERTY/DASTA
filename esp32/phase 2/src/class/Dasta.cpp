#include "Dasta.hpp"
#define max(a, b) ((a) > (b) ? (a) : (b))


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
    communication.start(); // wait for connection
    sensors.init();       // initialize the sensors (IMU)
    inited = true;
    
}
void Dasta::run_anguler_velocity_control(float time)
{
    // read the sensors
    // sensors.readSensors();

    // compute pid and write the control signal
    float Crx = pidRx.compute(communication.angular_velocity_command.data[0] + sensors.gyro.data[0], time);
    float Cry = pidRy.compute(communication.angular_velocity_command.data[1] + sensors.gyro.data[1], time);
    float Crz = pidRz.compute(communication.angular_velocity_command.data[2] - sensors.gyro.data[2], time);

    float c1 = Crx + Cry - Crz;
    float c2 = -Crx + Cry + Crz;
    float c3 = -Crx - Cry - Crz;
    float c4 = Crx - Cry + Crz;

    // Find the maximum absolute value among the commands
    // float max_command = max(max(max(abs(c1), abs(c2)), abs(c3)), abs(c4));

    // float relative_correction = 0.8; // 80% of the thrust maximum allowed for correction
    // // Scale the commands to ensure they stay within the range of [-r*thrust, 1.0-r*thrust]
    // if (max_command > 1.0 - relative_correction*thrust || max_command < -relative_correction*thrust) {
    //     float scale = (1.0 + relative_correction*thrust) / max(max_command, 1.0 - relative_correction*thrust);
    //     c1 *= scale;
    //     c2 *= scale;
    //     c3 *= scale;
    //     c4 *= scale;
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
    run_anguler_velocity_control(time);
    // // comput pid and write the control signal
    // float Crx = pidRx.compute(communication.rpy_command.data[0] - estimator.rpy.data[0], time);
    // float Cry = pidRy.compute(communication.rpy_command.data[1] - estimator.rpy.data[1], time);
    // float Crz = pidRz.compute(communication.angular_velocity_command.data[2] - sensors.gyro.data[2], time);

    // float c1 = Crx + Cry - Crz;
    // float c2 = -Crx + Cry + Crz;
    // float c3 = -Crx - Cry - Crz;
    // float c4 = Crx - Cry + Crz;

    // // Find the maximum absolute value among the commands
    // float max_command = max(max(max(abs(c1), abs(c2)), abs(c3)), abs(c4));

    // float relative_correction = 0.8; // 80% of the thrust maximum allowed for correction
    // // Scale the commands to ensure they stay within the range of [-r*thrust, 1.0-r*thrust]
    // if (max_command > 1.0 - relative_correction*thrust || max_command < -relative_correction*thrust) {
    //     float scale = (1.0 + relative_correction*thrust) / max(max_command, 1.0 - relative_correction*thrust);
    //     c1 *= scale;
    //     c2 *= scale;
    //     c3 *= scale;
    //     c4 *= scale;
    // }

    // actuators.motor1.write(c1 + thrust);
    // actuators.motor2.write(c2 + thrust);
    // actuators.motor3.write(c3 + thrust);
    // actuators.motor4.write(c4 + thrust);
}