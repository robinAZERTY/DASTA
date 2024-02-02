#ifndef CAM_HPP
#define CAM_HPP

#include "Quaternion.hpp"
#include "Led.hpp"

class Cam
{
public:
    Cam();
    ~Cam();

    Quaternion orientation;
    Vector position;
    data_type *k=nullptr;// constant in pixel*m/m
};

#endif // CAM_HPP
