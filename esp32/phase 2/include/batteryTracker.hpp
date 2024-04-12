#ifndef BATTERY_TRACKER_HPP
#define BATTERY_TRACKER_HPP

#include <Arduino.h>
#include "vector.hpp"

class BatteryTracker3s
{
    private :
    float last_time = 0;
    public:
    float delay = 1.0;
     uint8_t pinCell1, pinCell2, pinCell3, powerPin;
     float internal_resistance, current_consumption = 0;
     uint16_t _R1, _R2, _R3, _R4, _R5, _R6, _R7, _R8;

    float interpolate_charge_from_sample(float voltage);

     public:
     BatteryTracker3s(){};
     BatteryTracker3s(uint8_t pinCell1, uint8_t pinCell2, uint8_t pinCell3, uint8_t powerPin, uint16_t R1, uint16_t R2, uint16_t R3, uint16_t R4, uint16_t R5, uint16_t R6, uint16_t R7, uint16_t R8, float internal_resistance = 0.02);
     ~BatteryTracker3s();
     void init();
     void read();
     bool run(float now = millis()/1000.0);
     static float charge_sample[][2];
     static uint8_t sample_size;
     void set_current_consumption(float current_consumption){this->current_consumption = current_consumption;}
     float charge(uint8_t cell_index = 0);
     Vector voltages = Vector(4);
     Vector charges = Vector(4);
};
#endif // BATTERY_TRACKER_HPP