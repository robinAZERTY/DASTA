#include "Cam.hpp"

Cam::Cam()
{
}

Cam::~Cam()
{
}

void Cam::project(Vector res, const Vector &led_pos, const Quaternion &drone_orien, const Vector &drone_pos)
{
    rotate(res, drone_orien, led_pos);
    vector::add(res, res, drone_pos); // position of led in world frame
    vector::sub(res, res, position);

    orientation.conjugate();
    rotate(res, orientation, res); // position of led in cam frame
    orientation.conjugate();

    res.data[0] = res.data[0] * k / res.data[2];
    res.data[1] = res.data[1] * k / res.data[2]; // position of led in the image
}