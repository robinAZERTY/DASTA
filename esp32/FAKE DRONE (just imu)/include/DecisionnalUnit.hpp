#ifndef DECISIONNAL_UNIT_HPP
#define DECISIONNAL_UNIT_HPP

#include <Arduino.h>

class DecisionnalUnit
{

public:
    uint8_t user_event;
    DecisionnalUnit(/* args */);
    ~DecisionnalUnit();
};

#endif // DECISIONNAL_UNIT_HPP