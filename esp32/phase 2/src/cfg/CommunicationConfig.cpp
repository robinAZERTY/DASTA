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
    BL_stream::types.UNSIGNED_LONG = 'L';

    communication.send_stream.name = "send_stream:";
    communication.receive_stream.name = "receive_stream:";

    communication.send_stream.end_line = "end_line\n";
    communication.receive_stream.end_line = "end_line\n";

    communication.send_stream.include("time_us", (uint8_t *)&sensors.imu.getTime(), BL_stream::types.UNSIGNED_LONG_LONG, sizeof(sensors.imu.getTime()), true);
    communication.send_stream.include("gyro_raw", sensors.gyro_raw);
    communication.send_stream.include("acc_raw", sensors.acc_raw);
    communication.send_stream.include("gyro", sensors.gyro);
    communication.send_stream.include("acc", sensors.acc);
    communication.send_stream.include("orientation_rpy_leveled", estimator.rpy_leveled);
    communication.send_stream.include("orientation_rpy", estimator.rpy);
    communication.send_stream.include("orientation_q", estimator.orientation);
    communication.send_stream.include("velocity", estimator.velocity);
    communication.send_stream.include("position", estimator.position);

    communication.send_stream.include("motor_speeds", actuators.motor_speeds);
    communication.send_stream.include("angular_vel_command", angular_velocity_command);
    communication.send_stream.include("thrust_command", (uint8_t *)&thrust, BL_stream::types.FLOAT, sizeof(thrust));
    communication.send_stream.include("rpy_command", rpy_command);

    communication.send_stream.include("battery_voltages", sensors.LiPo.voltages);
    communication.send_stream.include("battery_lvl", sensors.LiPo.charges);
    communication.send_stream.include("internal_event", (uint8_t *)&decisionnal_unit.internal_event, BL_stream::types.UNSIGNED_CHAR, sizeof(decisionnal_unit.internal_event));

    communication.send_stream.include("kalman_delay_s", kalman_delay);
    communication.send_stream.include("close_loop_delay_s", close_loop_delay);
    communication.send_stream.include("stream_delay_s", stream_delay);


    communication.receive_stream.include("send_stream_register", (uint8_t *)&communication.send_stream._register, BL_stream::types.UNSIGNED_LONG, sizeof(communication.send_stream._register));
    communication.receive_stream.include("user_event", (uint8_t *)&decisionnal_unit.user_event, BL_stream::types.UNSIGNED_CHAR, sizeof(decisionnal_unit.user_event));
    communication.receive_stream.include("remote_commands", remote_commands);
    communication.receive_stream.include("remote_angular_velocity_sensitive", (uint8_t *)&remote_angular_velocity_sensitive, BL_stream::types.FLOAT, sizeof(remote_angular_velocity_sensitive));
    communication.receive_stream.include("remote_yaw_sensitive", (uint8_t *)&remote_yaw_sensitive, BL_stream::types.FLOAT, sizeof(remote_yaw_sensitive));
    communication.receive_stream.include("remote_attitude_sensitive", (uint8_t *)&remote_attitude_sensitive, BL_stream::types.FLOAT, sizeof(remote_attitude_sensitive));
    communication.receive_stream.include("send_stream_delay_ms", (uint8_t *)&communication.send_stream.delay, BL_stream::types.INT, sizeof(communication.send_stream.delay));
    communication.receive_stream.include("acc_bias_co", sensors.acc_bias_co);
    communication.receive_stream.include("gyro_bias_co", sensors.gyro_bias_co);
    communication.receive_stream.include("acc_scale_co", sensors.acc_scale_co);
    communication.receive_stream.include("gyro_scale_co", sensors.gyro_scale_co);

    communication.send_stream.delay = 100;
}
