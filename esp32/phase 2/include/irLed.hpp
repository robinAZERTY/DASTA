#ifndef IR_LED_HPP
#define IR_LED_HPP
#include "vector.hpp"
class IrLed
{
    public:
        Vector position = Vector(3);
        int pin;
        IrLed(){}; // default constructor
        IrLed(int pin, data_type x_pos, data_type y_pos, data_type z_pos)
        {
            this->pin = pin;
            position.data[0] = x_pos;
            position.data[1] = y_pos;
            position.data[2] = z_pos;
        }
        ~IrLed()
        {
            off();
        }
        void init()
        {
            pinMode(pin, OUTPUT);
            digitalWrite(pin, LOW);
        }
        void on()
        {
            digitalWrite(pin, HIGH);
        }
        void off(){
            digitalWrite(pin, LOW);
        }
};

#endif