#include "StateEstimate.hpp"

StateEstimate::StateEstimate()
{
    rpy.fill(0);
}

StateEstimate::~StateEstimate()
{
}

void StateEstimate::run(const float time)
{
    if (!running)
        return;
    if (last_time_proprio > 0)
    {
        *dt_proprio = time - last_time_proprio;
        // if (*dt_proprio > 0.015)
        // {
        //     Serial.println("warning : dt proprio too high (dt = " + String(*dt_proprio) + ")");
        // }
        ekf.predict();
    }
    last_time_proprio = time;

    for (uint_fast8_t i = 0; i < ekf.getZNumber(); i++)
        ekf.update(i);
    q2rpy(rpy, orientation);
}

void StateEstimate::initFromAccel(const Vector &accel)
{
    // compute roll and pitch from accelerometer
    rpy(2) = atan2(-accel.data[0], sqrt(accel.data[1] * accel.data[1] + accel.data[2] * accel.data[2]));
    rpy(1) = atan2(accel.data[1], accel.data[2]);
    rpy2q(orientation, rpy);
}