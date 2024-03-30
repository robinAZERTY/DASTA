#include "Dasta.hpp"

void Dasta::configCommunication()
{   
    BL_stream::types.CHAR = 'c';
    BL_stream::types.INT = 'i';
    BL_stream::types.UNSIGNED_LONG_LONG = 'Q';
    BL_stream::types.FLOAT = 'f';
    BL_stream::types.DOUBLE = 'd';
    BL_stream::types.VECTOR = 'v';
    BL_stream::types.MATRIX = 'm';
    BL_stream::types.UNSIGNED_CHAR = 'B';

    communication.send_stream.name = "send_stream:";
    communication.receive_stream.name = "receive_stream:";

    communication.send_stream.end_line = "end_line\n";
    communication.receive_stream.end_line = "end_line\n";
    
    communication.send_stream.include("time", (uint8_t *)&sensors.imu.getTime(), BL_stream::types.UNSIGNED_LONG_LONG, sizeof(sensors.imu.getTime()),true); 
    // communication.send_stream.include("acc", sensors.acc);
    communication.send_stream.include("gyro", sensors.gyro);
    // communication.send_stream.include("mag", sensors.mag);
    // communication.send_stream.include("position", estimator.position);
    // communication.send_stream.include("velocity", estimator.velocity);
    // communication.send_stream.include("orientation", estimator.orientation);
    // communication.send_stream.include("Covariance", *estimator.ekf->P);


    communication.receive_stream.include("angular_velocity_command", communication.angular_velocity_command);
    // communication.receive_stream.include("acc_bias", sensors.acc_bias_co);
    // communication.receive_stream.include("gyro_bias", sensors.gyro_bias_co);
    // communication.receive_stream.include("mag_bias", sensors.mag_bias_co);
    // communication.receive_stream.include("acc_scale", sensors.acc_scale_co);
    // communication.receive_stream.include("gyro_scale", sensors.gyro_scale_co);
    // communication.receive_stream.include("mag_scale", sensors.mag_scale_co);
    // communication.receive_stream.include("led1_pos", actuators.led1.position);
    // communication.receive_stream.include("led2_pos", actuators.led2.position);
    // communication.receive_stream.include("cam1_pos", sensors.cam1.position);
    // communication.receive_stream.include("cam1_or", sensors.cam1.orientation);
    // communication.receive_stream.include("cam1_k", (uint8_t *)&sensors.cam1.k, BL_stream::types.FLOAT, sizeof(sensors.cam1.k));
    // communication.receive_stream.include("cam2_pos", sensors.cam2.position);
    // communication.receive_stream.include("cam2_or", sensors.cam2.orientation);
    // communication.receive_stream.include("cam2_k", (uint8_t *)&sensors.cam2.k, BL_stream::types.FLOAT, sizeof(sensors.cam2.k));
    communication.receive_stream.include("user_event", (uint8_t *)&decisionnal_unit.user_event, BL_stream::types.UNSIGNED_CHAR, sizeof(decisionnal_unit.user_event));
    communication.receive_stream.include("send_stream_delay", (uint8_t *)&communication.send_stream.delay, BL_stream::types.INT, sizeof(communication.send_stream.delay));

    communication.send_stream.delay = 1000;
}
