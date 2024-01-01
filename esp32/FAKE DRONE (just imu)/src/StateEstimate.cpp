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

    ekf->x->set_zero();
    (*ekf->x)(6) = 1; // quaternion unitaire
    ekf->P->set_eye();
    // mul(*ekf->P, *ekf->P, 0.01);
    ekf->Q->set_zero();
    (*ekf->Q)(0, 0) = GYRO_VAR;
    (*ekf->Q)(1, 1) = GYRO_VAR;
    (*ekf->Q)(2, 2) = GYRO_VAR;
    (*ekf->Q)(3, 3) = ACC_VAR;
    (*ekf->Q)(4, 4) = ACC_VAR;
    (*ekf->Q)(5, 5) = ACC_VAR;

    ekf->R->set_zero();
    (*ekf->R)(0, 0) = EXT_VAR;
    (*ekf->R)(1, 1) = EXT_VAR;
    (*ekf->R)(2, 2) = EXT_VAR;
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

    if (true)
    {
        ekf->z->set_zero();
        ekf->update();
        last_time_extero = time;
    }
}