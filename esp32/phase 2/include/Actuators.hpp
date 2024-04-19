#ifndef ACTUATOR_HPP
#define ACTUATOR_HPP

#include "irLed.hpp"
#include "ESC.hpp"
#include "vector.hpp"

class Actuators
{
public:
    Actuators();
    ~Actuators();

    IrLed led1, led2, led3, led4;
    ESC motor1, motor2, motor3, motor4;
    Vector motor_speeds = Vector(4);
    void stopMotors();
    void engageMotors();

};

#endif // ACTUATOR_HPP