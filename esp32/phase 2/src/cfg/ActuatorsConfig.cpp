#include "Dasta.hpp"

void Dasta::configActuators()
{
    actuators.motor1.attach(2);
    actuators.motor2.attach(0);
    actuators.motor3.attach(4);
    actuators.motor4.attach(15);
    actuators.led1.pin = 17;
    actuators.led1.position.data[0] = 0.1351;
    actuators.led1.position.data[1] = -0.138001;
    actuators.led1.position.data[2] = -0.017906;
    actuators.led2.pin = 5;
    actuators.led2.position.data[0] = 0.1351;
    actuators.led2.position.data[1] = 0.138001;
    actuators.led2.position.data[2] = -0.017906;
    actuators.led3.pin = 18;
    actuators.led3.position.data[0] = -0.1365;
    actuators.led3.position.data[1] = 0.138;
    actuators.led3.position.data[2] = -0.012143;
    actuators.led4.pin = 16;
    actuators.led4.position.data[0] = -0.1365;
    actuators.led4.position.data[1] = -0.138;
    actuators.led4.position.data[2] = -0.012143;

    actuators.led1.init();
    actuators.led2.init();
    actuators.led3.init();
    actuators.led4.init();

    while (!actuators.motor1.runArm() || !actuators.motor2.runArm() || !actuators.motor3.runArm() || !actuators.motor4.runArm())
        delay(10);

    actuators.stopMotors();
    pidRx = Pid(0.015, 0.03, 0.001, 0.02, 0.05, -0.1, 0.1);    
    pidRy = Pid(0.015, 0.03, 0.001, 0.02, 0.05, -0.1, 0.1);
    pidRz = Pid(0.15, 0.1, 0.0, 0.1, 0, -0.2, 0.2); 
    thrust = 0.0; 
    pidRoll = Pid(10, 0, 0.0, 0, 0.0, -12, 12);
    pidPitch = Pid(10, 0, 0.0, 0, 0.0, -12, 12);
    pidYaw = Pid(10, 0, 0.0, 0, 0.0, -12, 12);
    estimator.lpf_gyr_derx.setTimeConstant(pidRx.timeConstDerFilter);
    estimator.lpf_gyr_dery.setTimeConstant(pidRy.timeConstDerFilter);

}