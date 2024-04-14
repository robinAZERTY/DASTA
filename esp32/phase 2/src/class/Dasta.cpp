#include "Dasta.hpp"
#define max(a, b) ((a) > (b) ? (a) : (b))

Dasta::Dasta()
{
    angular_velocity_command.fill(0);
    last_angular_velocity_command.fill(0);
    angular_velocity_command_der.fill(0);
    remote_commands.fill(0);
    rpy_command.fill(0);
    angular_velocity_command.fill(0);
    kalman_delay.fill(0);
    close_loop_delay.fill(0);
    stream_delay.fill(0);
}
void Dasta::init()
{
    configureStateEstimate();
    configSensorPreProcessing();
    configCommunication();
    configActuators();
    communication.device_name = "ESP32-Bluetooth";
    communication.start(); // wait for connection
    sensors.init();        // initialize the sensors (IMU)

    inited = true;
}
void Dasta::run_angular_velocity_control(float dt)
{
    if (!eq(last_angular_velocity_command, 0) && dt > 1e-6)
    {
        sub(angular_velocity_command_der, angular_velocity_command, last_angular_velocity_command);
        mul(angular_velocity_command_der, angular_velocity_command_der, 1 / dt);
    }
    cd(last_angular_velocity_command, angular_velocity_command);

    // compute pid and write the control signal
    float Crx = pidRx.compute_using_external_derivative(angular_velocity_command.data[0] - sensors.gyro.data[0], angular_velocity_command_der.data[0] - estimator.lpf_gyr_derx.getFilteredValue(), dt);
    float Cry = pidRy.compute_using_external_derivative(angular_velocity_command.data[1] - sensors.gyro.data[1], angular_velocity_command_der.data[1] - estimator.lpf_gyr_dery.getFilteredValue(), dt);
    float Crz = pidRz.compute(angular_velocity_command.data[2] - sensors.gyro.data[2], dt);

    float c1 = Crx + Cry - Crz;
    float c2 = -Crx + Cry + Crz;
    float c3 = -Crx - Cry - Crz;
    float c4 = Crx - Cry + Crz;

    actuators.motor1.write(c1 + thrust);
    actuators.motor2.write(c2 + thrust);
    actuators.motor3.write(c3 + thrust);
    actuators.motor4.write(c4 + thrust);

    actuators.motor_speeds.data[0] = actuators.motor1.read();
    actuators.motor_speeds.data[1] = actuators.motor2.read();
    actuators.motor_speeds.data[2] = actuators.motor3.read();
    actuators.motor_speeds.data[3] = actuators.motor4.read();
}

void Dasta::compute_angular_velocity_command_for_attitude_control(float dt)
{
    angular_velocity_command.data[0] = pidRoll.compute(rpy_command.data[0] - estimator.rpy_leveled.data[0], dt);
    angular_velocity_command.data[1] = pidPitch.compute(rpy_command.data[1] - estimator.rpy_leveled.data[1], dt);
    angular_velocity_command.data[2] = pidYaw.compute(rpy_command.data[2] - estimator.rpy.data[2], dt);
}

void Dasta::run_attitude_control(float dt)
{
    compute_angular_velocity_command_for_attitude_control(dt);
    this->run_angular_velocity_control(dt);
}

void Dasta::control(const float dt)
{
    switch (control_mode)
    {
    case ControlMode::ANGULAR_VELOCITY:
        angular_velocity_command.data[0] = remote_commands.data[0] * remote_angular_velocity_sensitive;
        angular_velocity_command.data[1] = remote_commands.data[1] * remote_angular_velocity_sensitive;
        angular_velocity_command.data[2] = remote_commands.data[2] * remote_angular_velocity_sensitive;
        thrust = remote_commands.data[3];
        run_angular_velocity_control(dt);
        break;
    case ControlMode::ATTITUDE:
        rpy_command.data[0] = remote_commands.data[0] * remote_attitude_sensitive;
        rpy_command.data[1] = remote_commands.data[1] * remote_attitude_sensitive;
        compute_angular_velocity_command_for_attitude_control(dt);
        angular_velocity_command.data[2] = remote_commands.data[2] * remote_yaw_sensitive;
        thrust = remote_commands.data[3];
        run_angular_velocity_control(dt);
        break;
    case ControlMode::VELOCITY:
        break;
    case ControlMode::POSITION:
        break;
    default:
        break;
    }
}