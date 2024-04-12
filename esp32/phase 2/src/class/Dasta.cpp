#include "Dasta.hpp"
#define max(a, b) ((a) > (b) ? (a) : (b))


Dasta::Dasta()
{
}
void Dasta::init()
{
    configureStateEstimate();
    configSensorPreProcessing();
    configCommunication();
    configActuators();
    communication.device_name = "ESP32-Bluetooth";
    communication.start(); // wait for connection
    sensors.init();       // initialize the sensors (IMU)
    inited = true;
    
}
void Dasta::run_angular_velocity_control(float time)
{
    if (!angular_velocity_control_running)
        return;
    // compute pid and write the control signal
    float Crx = pidRx.compute(angular_velocity_command.data[0] - sensors.gyro.data[0], time);
    float Cry = pidRy.compute(angular_velocity_command.data[1] - sensors.gyro.data[1], time);
    float Crz = pidRz.compute(angular_velocity_command.data[2] - sensors.gyro.data[2], time);

    float c1 = Crx + Cry - Crz;
    float c2 = -Crx + Cry + Crz;
    float c3 = -Crx - Cry - Crz;
    float c4 = Crx - Cry + Crz;

    // // Find the maximum absolute value among the commands
    // float max_command = max(max(max(abs(c1), abs(c2)), abs(c3)), abs(c4));

    // float relative_correction = 0.8; // 80% of the thrust maximum allowed for correction
    // // Scale the commands to ensure they stay within the range of [-r*thrust, 1.0-r*thrust]
    // if (max_command > 1.0 - relative_correction*thrust || max_command < -relative_correction*thrust) {
    //     float scale = (1.0 + relative_correction*thrust) / max(max_command, 1.0 - relative_correction*thrust);
    //     c1 *= scale;
    //     c2 *= scale;
    //     c3 *= scale;
    //     c4 *= scale;
    // }

    actuators.motor1.write(c1 + thrust);
    actuators.motor2.write(c2 + thrust);
    actuators.motor3.write(c3 + thrust);
    actuators.motor4.write(c4 + thrust);
}

void Dasta::run_attitude_control(float time)
{
    if (!attitude_control_running)
        return;



    // data_type plate_angle = sqrt(communication.rpy_command.data[0] * communication.rpy_command.data[0] + communication.rpy_command.data[1] * communication.rpy_command.data[1]);
    // // Serial.println("plate_angle : "+String(plate_angle));
    // Vector axis(3);
    // if (abs(plate_angle)<1e-6)
    // {
    //     axis.data[0] = 0;
    //     axis.data[1] = 0;
    //     axis.data[2] = 1;
    // }
    // else
    // {
    //     axis.data[0] = communication.rpy_command.data[0] / plate_angle;
    //     axis.data[1] = communication.rpy_command.data[1] / plate_angle;
    //     axis.data[2] = 0;
    // }
    // // Serial.println("axis : "+String(axis.data[0])+" "+String(axis.data[1])+" "+String(axis.data[2]));
    // Quaternion q1(axis, plate_angle);
    // // Serial.println("q1 : "+String(q1.data[0])+" "+String(q1.data[1])+" "+String(q1.data[2])+" "+String(q1.data[3]));
    // Vector tmp(3);
    // tmp.data[0] = 1;
    // tmp.data[1] = 0;
    // tmp.data[2] = 0;
    // // Serial.println("current_orientation : "+String(estimator.orientation.data[0])+" "+String(estimator.orientation.data[1])+" "+String(estimator.orientation.data[2])+" "+String(estimator.orientation.data[3])+" ");
    // rotate(axis,estimator.orientation,tmp);
    // data_type current_yaw = atan2(axis.data[1],axis.data[0]);
    // // Serial.println("current_yaw : "+String(current_yaw));


    // tmp.data[0] = 0;
    // tmp.data[1] = 0;
    // tmp.data[2] = 1;
    // // Serial.println("tmp : "+String(tmp.data[0])+" "+String(tmp.data[1])+" "+String(tmp.data[2]));
    // Quaternion q_yaw(tmp,current_yaw);
    // // Serial.println("q_yaw : "+String(q_yaw.data[0])+" "+String(q_yaw.data[1])+" "+String(q_yaw.data[2])+" "+String(q_yaw.data[3]));
    // Quaternion q_goal;
    // mul(q_goal,q_yaw,q1);
    // // Serial.println("q_goal : "+String(q_goal.data[0])+" "+String(q_goal.data[1])+" "+String(q_goal.data[2])+" "+String(q_goal.data[3]));
    // Quaternion q2;
    // Quaternion q3;
    // data_type forward = 0.05;
    // data_type step = 0.001;
    // slerp(q2,estimator.orientation,q_goal,forward-step);
    // // Serial.println("current_orientation : "+String(estimator.orientation.data[0])+" "+String(estimator.orientation.data[1])+" "+String(estimator.orientation.data[2])+" "+String(estimator.orientation.data[3])+" ");
    // // Serial.println("q_goal : "+String(q_goal.data[0])+" "+String(q_goal.data[1])+" "+String(q_goal.data[2])+" "+String(q_goal.data[3]));
    // slerp(q3,estimator.orientation,q_goal,forward+step);
    // // Serial.println("q2 : "+String(q2.data[0])+" "+String(q2.data[1])+" "+String(q2.data[2])+" "+String(q2.data[3]));
    // // Serial.println("q3 : "+String(q3.data[0])+" "+String(q3.data[1])+" "+String(q3.data[2])+" "+String(q3.data[3]));
    // sub(q1,q3,q2);
    // // Serial.println("q1 : "+String(q1.data[0])+" "+String(q1.data[1])+" "+String(q1.data[2])+" "+String(q1.data[3]));
    // mul(q1,q1,1.0/(step*2.0));
    // // Serial.println("q1 : "+String(q1.data[0])+" "+String(q1.data[1])+" "+String(q1.data[2])+" "+String(q1.data[3]));
    // mul(q_yaw,q1,estimator.orientation);
    // // Serial.println("q_yaw : "+String(q_yaw.data[0])+" "+String(q_yaw.data[1])+" "+String(q_yaw.data[2])+" "+String(q_yaw.data[3]));
    // mul(q_yaw,q_yaw,2);
    // // Serial.println("q_yaw : "+String(q_yaw.data[0])+" "+String(q_yaw.data[1])+" "+String(q_yaw.data[2])+" "+String(q_yaw.data[3]));

    // tmp.data[0] = 0;
    // tmp.data[1] = 0;
    // tmp.data[2] = 1;
    // rotate(axis,estimator.orientation,tmp);
    // data_type up = axis.data[2];
    // data_type compensated_thrust = thrust/up;
    // angular_velocity_command.data[0] = pidRoll.compute(q_yaw.data[1], time);
    // angular_velocity_command.data[1] = pidPitch.compute(q_yaw.data[2], time);
    // angular_velocity_command.data[2] = pidYaw.compute(q_yaw.data[3], time) + communication.angular_velocity_command.data[2];
    angular_velocity_command.data[0] = pidRoll.compute(communication.rpy_command.data[0]-estimator.rpy.data[0]+0.04, time);
    angular_velocity_command.data[1] = pidPitch.compute(communication.rpy_command.data[1] - estimator.rpy.data[1]-0.03, time);
    angular_velocity_command.data[2] = communication.angular_velocity_command.data[2];

    this->run_angular_velocity_control(time);
}