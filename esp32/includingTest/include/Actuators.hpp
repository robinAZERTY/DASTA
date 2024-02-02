#ifndef ACTUATOR_HPP
#define ACTUATOR_HPP

#include "Led.hpp"

class Actuators
{
public:
    Actuators();
    ~Actuators();

    Led *led;
    uint_fast8_t led_num=0;
};

#endif // ACTUATOR_HPP