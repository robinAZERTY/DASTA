#include "StateEstimate.hpp"

StateEstimate::StateEstimate()
{
    position.size = 3;
    velocity.size = 3;
}

StateEstimate::~StateEstimate()
{
    delete ekf;
}

void StateEstimate::run(const float time)
{
    if (!running)
        return;

    if (last_time_proprio > 0)
    {
        dt_proprio = time - last_time_proprio;
        ekf->predict();
    }
    last_time_proprio = time;

    for (uint_fast8_t i = 0; i < ekf->z_number; i++)
    {
        if (ekf->z_available[i])
        {
            ekf->update(i);
            ekf->z_available[i] = false;
        }
    }
}