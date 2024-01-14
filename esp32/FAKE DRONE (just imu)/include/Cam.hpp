#ifndef CAM_HPP
#define CAM_HPP

#include "quaternion.hpp"

class Cam
{
public:
    Cam();
    ~Cam();

    Quaternion orientation;
    Vector position;

    Vector
};

#endif // CAM_HPP
