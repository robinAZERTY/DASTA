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
    communication.send_stream.include("gyro_raw", sensors.gyro_raw,true);
    communication.send_stream.include("acc_raw", sensors.acc_raw,true);
    communication.send_stream.include("angular_vel_command", communication.angular_velocity_command,true);
    communication.send_stream.include("m1PWM", (uint8_t *)&actuators.motor1.read(), BL_stream::types.FLOAT, sizeof(actuators.motor1.read()),true);
    communication.send_stream.include("m2PWM", (uint8_t *)&actuators.motor2.read(), BL_stream::types.FLOAT, sizeof(actuators.motor2.read()),true);
    communication.send_stream.include("m3PWM", (uint8_t *)&actuators.motor3.read(), BL_stream::types.FLOAT, sizeof(actuators.motor3.read()),true);
    communication.send_stream.include("m4PWM", (uint8_t *)&actuators.motor4.read(), BL_stream::types.FLOAT, sizeof(actuators.motor4.read()),true);
    communication.send_stream.include("orientation", estimator.orientation,true);

    communication.receive_stream.include("angular_velocity_command", communication.angular_velocity_command);
    communication.receive_stream.include("thrust_command", (uint8_t *)&thrust, BL_stream::types.FLOAT, sizeof(thrust));
    communication.receive_stream.include("user_event", (uint8_t *)&decisionnal_unit.user_event, BL_stream::types.UNSIGNED_CHAR, sizeof(decisionnal_unit.user_event));
    communication.receive_stream.include("send_stream_delay", (uint8_t *)&communication.send_stream.delay, BL_stream::types.INT, sizeof(communication.send_stream.delay));
    communication.receive_stream.include("orientation", estimator.orientation);
    communication.receive_stream.include("acc_bias_co", sensors.acc_bias_co);
    communication.receive_stream.include("gyro_bias_co", sensors.gyro_bias_co);
    communication.receive_stream.include("acc_scale_co", sensors.acc_scale_co);
    communication.receive_stream.include("gyro_scale_co", sensors.gyro_scale_co);
    communication.receive_stream.include("rpy_command", communication.rpy_command);
    communication.send_stream.delay = 100;
}
