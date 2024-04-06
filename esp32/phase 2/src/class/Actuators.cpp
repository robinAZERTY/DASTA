#include "Actuators.hpp"

Actuators::Actuators()
{
}

Actuators::~Actuators()
{
}

void Actuators::stopMotors()
{
    motor1.disengage();
    motor2.disengage();
    motor3.disengage();
    motor4.disengage();
}