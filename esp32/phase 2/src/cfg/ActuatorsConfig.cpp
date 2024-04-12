#include "Dasta.hpp"

void Dasta::configActuators()
{
    actuators.motor1.attach(2);
    actuators.motor2.attach(0);
    actuators.motor3.attach(4);
    actuators.motor4.attach(15);
    while (!actuators.motor1.runArm() || !actuators.motor2.runArm() || !actuators.motor3.runArm() || !actuators.motor4.runArm())
        delay(10);

    actuators.stopMotors();
    // delay(1000);
    // actuators.motor1.write(0.001);
    // delay(1000);
    // actuators.motor1.write(0.0);
    // actuators.motor2.write(0.001);
    // delay(1000);
    // actuators.motor2.write(0.0);
    // actuators.motor3.write(0.001);
    // delay(1000);
    // actuators.motor3.write(0.0);
    // actuators.motor4.write(0.001);
    // delay(1000);
    // actuators.motor4.write(0.0);


    // turn a bit one by one
    // actuators.motor1.write(0.05);
    // delay(1000);
    // actuators.motor1.write(0.0);
    // actuators.motor2.write(0.05);
    // delay(1000);
    // actuators.motor2.write(0.0);
    // actuators.motor3.write(0.05);
    // delay(1000);
    // actuators.motor3.write(0.0);
    // actuators.motor4.write(0.05);
    // delay(1000);
    // actuators.motor4.write(0.0);
    // thrust = 0.2;
    pidRx = Pid(0.03, 0.1, 0.004, 0.1, 0.02, -0.1, 0.1);

    pidRy = Pid(0.03, 0.1, 0.004, 0.1, 0.02, -0.1, 0.1);
    pidRz = Pid(0.15, 0.1, 0.0, 0.2, 0, -0.2, 0.2); 
    thrust = 0.0; 
    pidRoll = Pid(5, 0, 0.0, 0, 0.0, -6, 6);
    pidPitch = Pid(5, 0, 0.0, 0, 0.0, -6, 6);
    pidYaw = Pid(5, 0, 0.0, 0, 0.0, -6, 6);
    // pidRx = Pid(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    // pidRy = Pid(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    // pidRz = Pid(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);   
}