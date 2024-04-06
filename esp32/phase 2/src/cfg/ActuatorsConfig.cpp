#include "Dasta.hpp"

void Dasta::configActuators()
{
    actuators.motor1.attach(33);
    actuators.motor2.attach(25);
    actuators.motor3.attach(26);
    actuators.motor4.attach(32);
    while (!actuators.motor1.runArm() || !actuators.motor2.runArm() || !actuators.motor3.runArm() || !actuators.motor4.runArm())
        delay(10);

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
    pidRx = Pid(0.05, 0.05, 0.1, 0.1, 0.05, -0.1, 0.1);
    pidRy = Pid(0.05, 0.05, 0.1, 0.1, 0.05, -0.1, 0.1);
    pidRz = Pid(0.17, 0.2, 0.0, 0, 0, -0.2, 0.2); 
    thrust = 0.0; 
    // pidRx = Pid(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    // pidRy = Pid(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    // pidRz = Pid(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);   
}