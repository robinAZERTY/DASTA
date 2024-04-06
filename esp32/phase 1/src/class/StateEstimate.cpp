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
    {
        ekf.update(i);
    }
    q2rpy(rpy, orientation);
}