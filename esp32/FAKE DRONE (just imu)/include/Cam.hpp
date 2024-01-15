#ifndef CAM_HPP
#define CAM_HPP

#include "quaternion.hpp"
#include "led.hpp"

class Cam
{
public:
    Cam();
    ~Cam();

    Quaternion orientation;
    Vector position;
    data_type k;// constant in pixel*m/m

    Led *leds;
    uint_fast8_t led_num;
};

#endif // CAM_HPP
