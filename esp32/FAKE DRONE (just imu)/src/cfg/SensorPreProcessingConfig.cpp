#include "Dasta.hpp"

void Dasta::configSensorPreProcessing()
{
    // allocate and link the matrices and vectors
    sensors.acc_bias_co.data = new data_type[3];
    sensors.gyro_bias_co.data = new data_type[3];

    sensors.acc_scale_co.data = new data_type[9];
    sensors.gyro_scale_co.data = new data_type[9];


    sensors.acc_bias_co.fill(0);
    sensors.gyro_bias_co.fill(0);

    sensors.acc_scale_co.set_eye();
    sensors.gyro_scale_co.set_eye();


    // (sensors.acc_bias_co = Vector(3)).fill(0);
    // (sensors.gyro_bias_co = Vector(3)).fill(0);
    // (sensors.mag_bias_co = Vector(3)).fill(0);

    // (sensors.acc_scale_co = Matrix(3, 3)).set_eye();
    // (sensors.gyro_scale_co = Matrix(3, 3)).set_eye();
    // (sensors.mag_scale_co = Matrix(3, 3)).set_eye();

    // sensors.gyro.data = estimator.ekf->u->data;
    // sensors.acc.data = estimator.ekf->u->data + 3;
    // sensors.acc.data = new data_type[3];
    // sensors.gyro.data = new data_type[3];
}