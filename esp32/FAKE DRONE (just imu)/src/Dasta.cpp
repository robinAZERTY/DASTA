#include "Dasta.hpp"

Dasta::Dasta()
{
    // allocate and link the matrices and vectors
    (sensors.acc_bias = Vector(3)).set_zero();
    (sensors.gyro_bias = Vector(3)).set_zero();
    (sensors.mag_bias = Vector(3)).set_zero();

    (sensors.acc_scale = Matrix(3, 3)).set_eye();
    (sensors.gyro_scale = Matrix(3, 3)).set_eye();
    (sensors.mag_scale = Matrix(3, 3)).set_eye();

    sensors.gyro.data = estimator.ekf->u->data;
    sensors.acc.data = estimator.ekf->u->data + 3;
    sensors.mag = Vector(3);

    communication.send_stream.include("time", (uint8_t *)&sensors.getTime(), UNSIGNED_LONG_LONG_KEY, sizeof(sensors.getTime())); 
    communication.send_stream.include("acc", sensors.acc);
    communication.send_stream.include("gyro", sensors.gyro);
    communication.send_stream.include("mag", sensors.mag);
    communication.send_stream.include("position", estimator.position);
    communication.send_stream.include("velocity", estimator.velocity);
    communication.send_stream.include("orientation", estimator.orientation);
    // communication.send_stream.include("Covariance", *estimator.ekf->P);

    communication.receive_stream.include("acc_bias", sensors.acc_bias);
    communication.receive_stream.include("gyro_bias", sensors.gyro_bias);
    communication.receive_stream.include("mag_bias", sensors.mag_bias);
    communication.receive_stream.include("acc_scale", sensors.acc_scale);
    communication.receive_stream.include("gyro_scale", sensors.gyro_scale);
    communication.receive_stream.include("mag_scale", sensors.mag_scale);

    cd(estimator.ekf->x->data + 10,sensors.gyro_bias);
    cd(estimator.ekf->x->data + 13,sensors.acc_bias);
    sensors.gyro_bias.data = estimator.ekf->x->data + 10;
    sensors.acc_bias.data = estimator.ekf->x->data + 13;
    sensors.gyro_scale.data = estimator.ekf->x->data + 16;
    sensors.acc_scale.data = estimator.ekf->x->data + 25;
}