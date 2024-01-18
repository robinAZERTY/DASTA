#include "Cam.hpp"

Cam::Cam()
{
    orientation = Quaternion(1, 0, 0, 0);
    position = Vector(3);
}

Cam::~Cam()
{
}