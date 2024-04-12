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

void Actuators::engageMotors()
{
    motor1.engage();
    motor2.engage();
    motor3.engage();
    motor4.engage();
}