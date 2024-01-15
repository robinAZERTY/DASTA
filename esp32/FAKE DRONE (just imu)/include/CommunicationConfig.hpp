/*
*/
#ifndef COMMUNICATION_CONFIG_HPP
#define COMMUNICATION_CONFIG_HPP
#include "Dasta.hpp"

// types keys
#define CHAR_KEY 'c'
#define INT_KEY 'i'
#define UNSIGNED_LONG_LONG_KEY 'Q'
#define FLOAT_KEY 'f'
#define DOUBLE_KEY 'd'
#define VECTOR_KEY 'v'
#define MATRIX_KEY 'm'

#define SEND_STREAM_NAME "send_stream:"
#define RECEIVE_STREAM_NAME "receive_stream:"
#define END_LINE "end_line\n"

void Dasta::configCommunication()
{
    communication.send_stream.include("time", (uint8_t *)&sensors.getTime(), UNSIGNED_LONG_LONG_KEY, sizeof(sensors.getTime())); 
    communication.send_stream.include("acc", sensors.acc, false);
    communication.send_stream.include("gyro", sensors.gyro,false);
    communication.send_stream.include("mag", sensors.mag,false);
    communication.send_stream.include("position", estimator.position,false);
    communication.send_stream.include("velocity", estimator.velocity,false);
    communication.send_stream.include("orientation", estimator.orientation,false);
    // communication.send_stream.include("Covariance", *estimator.ekf->P);

    communication.receive_stream.include("acc_bias", sensors.acc_bias);
    communication.receive_stream.include("gyro_bias", sensors.gyro_bias);
    communication.receive_stream.include("mag_bias", sensors.mag_bias);
    communication.receive_stream.include("acc_scale", sensors.acc_scale);
    communication.receive_stream.include("gyro_scale", sensors.gyro_scale);
    communication.receive_stream.include("mag_scale", sensors.mag_scale);
    communication.receive_stream.include("led1_pos", actuators.led1.position);
    communication.receive_stream.include("led2_pos", actuators.led2.position);
    communication.receive_stream.include("cam1_pos", sensors.cam1.position);
    communication.receive_stream.include("cam1_or", sensors.cam1.orientation);
    communication.receive_stream.include("cam1_k", (uint8_t *)&sensors.cam1.k, FLOAT_KEY, sizeof(sensors.cam1.k));
    communication.receive_stream.include("cam2_pos", sensors.cam2.position);
    communication.receive_stream.include("cam2_or", sensors.cam2.orientation);
    communication.receive_stream.include("cam2_k", (uint8_t *)&sensors.cam2.k, FLOAT_KEY, sizeof(sensors.cam2.k));
    communication.receive_stream.include("user_event", (uint8_t *)&decisionnal_unit.user_event, INT_KEY, sizeof(decisionnal_unit.user_event));

}

#endif // COMMUNICATION