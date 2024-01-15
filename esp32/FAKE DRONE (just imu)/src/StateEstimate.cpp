#include "StateEstimate.hpp"
#include "StateEstimateConfig.hpp"

float StateEstimate::dt_proprio = 0;

StateEstimate::StateEstimate()
{

    initEKF(ekf);
    position.size = 3;
    position.data = ekf->x->data;
    velocity.size = 3;
    velocity.data = ekf->x->data + 3;
    orientation.data = ekf->x->data + 6;
}

StateEstimate::~StateEstimate()
{
    delete ekf;
}

void StateEstimate::run(const float time)
{
    if (running)
        if (last_time_proprio > 0)
        {
            dt_proprio = time - last_time_proprio;
            ekf->predict();
        }

    last_time_proprio = time;

    // ekf->z->set_zero();
    // ekf->update();
    // last_time_extero = time;
}