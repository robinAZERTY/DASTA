#include "StateEstimate.hpp"
#include "StateEstimateConfig.hpp"

float StateEstimate::dt_proprio = 0;

StateEstimate::StateEstimate()
{
    ekf = new Ekf(f, h, X_DIM, Z_DIM, U_DIM, Fx, Fu, H);
    position.size = 3;
    position.data = ekf->x->data;
    velocity.size = 3;
    velocity.data = ekf->x->data + 3;
    orientation.data = ekf->x->data + 6;
    initXPRQ(*ekf->x, *ekf->P, *ekf->R, *ekf->Q);
}

StateEstimate::~StateEstimate()
{
    delete ekf;
}

void StateEstimate::run(const float time)
{

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