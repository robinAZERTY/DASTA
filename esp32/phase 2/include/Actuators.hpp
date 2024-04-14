#ifndef ACTUATOR_HPP
#define ACTUATOR_HPP

// #include "Led.hpp"
#include "ESC.hpp"
#include "vector.hpp"

class Actuators
{
public:
    Actuators();
    ~Actuators();

    // Led led1, led2;
    ESC motor1, motor2, motor3, motor4;
    Vector motor_speeds = Vector(4);
    void stopMotors();
    void engageMotors();

};

#endif // ACTUATOR_HPP