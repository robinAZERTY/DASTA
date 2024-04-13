#include "StateEstimate.hpp"

StateEstimate::StateEstimate()
{
    rpy.fill(0);
}

StateEstimate::~StateEstimate()
{
}

void StateEstimate::run(const float dt)
{
    if (!running)
        return;

    *dt_proprio = dt;
    if (lpf_gyr_derx.firstRun)
    {
        lpf_gyr_derx.filter(0, *dt_proprio);
        lpf_gyr_dery.filter(0, *dt_proprio);
        lpf_gyr_derz.filter(0, *dt_proprio);
    }
    else
    {
        float gyr_derx = (ekf.u.data[0] - last_gyro.data[0]) / *dt_proprio;
        float gyr_dery = (ekf.u.data[1] - last_gyro.data[1]) / *dt_proprio;
        float gyr_derz = (ekf.u.data[2] - last_gyro.data[2]) / *dt_proprio;
        cd(last_gyro, ekf.u);
        lpf_gyr_derx.filter(gyr_derx, *dt_proprio);
        lpf_gyr_dery.filter(gyr_dery, *dt_proprio);
        lpf_gyr_derz.filter(gyr_derz, *dt_proprio);
    }
    ekf.predict();

    for (uint_fast8_t i = 0; i < ekf.getZNumber(); i++)
        ekf.update(i);
    q2rpy(rpy, orientation);
}

void StateEstimate::initFromAccel(const Vector &accel)
{
    // compute roll and pitch from accelerometer
    rpy(0) = atan2(-accel.data[1], -accel.data[2]);
    rpy(1) = -atan2(-accel.data[0], sqrt(accel.data[1] * accel.data[1] + accel.data[2] * accel.data[2]));
    rpy(2) = 0;
    rpy2q(orientation, rpy);
}