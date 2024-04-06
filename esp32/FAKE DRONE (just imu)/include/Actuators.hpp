#ifndef ACTUATOR_HPP
#define ACTUATOR_HPP

#include "Led.hpp"

class Actuators
{
public:
    Actuators();
    ~Actuators();

    Led led1, led2;
};

#endif // ACTUATOR_HPP