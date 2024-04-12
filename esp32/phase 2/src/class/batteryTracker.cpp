#include "batteryTracker.hpp"

BatteryTracker3s::BatteryTracker3s(uint8_t pinCell1, uint8_t pinCell2, uint8_t pinCell3, uint8_t powerPin, uint16_t R1, uint16_t R2, uint16_t R3, uint16_t R4, uint16_t R5, uint16_t R6, uint16_t R7, uint16_t R8, float internal_resistance)
{
    this->pinCell1 = pinCell1;
    this->pinCell2 = pinCell2;
    this->pinCell3 = pinCell3;
    this->powerPin = powerPin;
    this->_R1 = R1;
    this->_R2 = R2;
    this->_R3 = R3;
    this->_R4 = R4;
    this->_R5 = R5;
    this->_R6 = R6;
    this->_R7 = R7;
    this->_R8 = R8;
    this->internal_resistance = internal_resistance;
}

BatteryTracker3s::~BatteryTracker3s()
{
}

void BatteryTracker3s::init()
{
    pinMode(pinCell1, INPUT);
    pinMode(pinCell2, INPUT);
    pinMode(pinCell3, INPUT);
    pinMode(powerPin, INPUT);
}

void BatteryTracker3s::read()
{
    float v1 = analogReadMilliVolts(pinCell1) * 0.001 * (_R1 + _R2) / _R2;
    float v2 = analogReadMilliVolts(pinCell2) * 0.001 * (_R3 + _R4) / _R4;
    float v3 = analogReadMilliVolts(pinCell3) * 0.001 * (_R5 + _R6) / _R6;
    float v4 = analogReadMilliVolts(powerPin) * 0.001 * (_R7 + _R8) / _R8;

    voltages.data[0] = v1;
    voltages.data[1] = v2-v1;
    voltages.data[2] = v3-v2;
    voltages.data[3] = v4;
    charge(0);
    charge(1);
    charge(2);
    charge(3);
}

float BatteryTracker3s::charge(uint8_t cell_index)
{
    if (cell_index == 0)
    {
        charges.data[cell_index] = (charges(1) + charge(2) + charge(3)) / 3;
       return charges.data[cell_index];
     }
    float drop_out_voltage = internal_resistance * current_consumption;
    float charge_voltage = voltages.data[cell_index] + drop_out_voltage;
    charges.data[cell_index] = interpolate_charge_from_sample(charge_voltage);
    return charges.data[cell_index];
}

float BatteryTracker3s::interpolate_charge_from_sample(float voltage)
{
    uint8_t i = 0;
    while (i < sample_size && charge_sample[i][1] < voltage)
        i++;
    if (i == 0)
        return 0;
    if (i == sample_size)
        return 1.0;
    float alpha = (voltage-charge_sample[i-1][1])/(charge_sample[i][1]-charge_sample[i-1][1]);
    return (1-alpha) * charge_sample[i-1][0] + alpha * charge_sample[i][0];
}

bool BatteryTracker3s::run(float now)
{
    if (now - last_time < delay)
    return false;

    last_time = now;
    read();
    return true;
}