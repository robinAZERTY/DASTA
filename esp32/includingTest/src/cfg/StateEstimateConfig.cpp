#include "Dasta.hpp"

/*
____________STATE ESTIMATE VECTOR______________

        State                   unit        index
postion(xyz)                    m           0:2
velocity(xyz)                   m/s         3:5
orientation(quaternion)                     6:9



___________COMMANDS VECTOR_____________________

        State                   unit        index
gyroscopes(xyz)                 rad/s       0:2
accelerometers(xyz)             m/s²        3:5


___________SENSORS VECTOR_____________________

        State                   unit       index
beaci_camj(uv)                  pixel       0:1

The number of measurements function z_num depends on the number of cameras and leds.
z_num = number_of_cameras * number_of_leds

___________PARAMS VECTOR_____________________

        State                   unit       index
dt                             s            0
g                              m/s²         1
.............leds positions...................
led1_pos(xyz)                   m           2:4
..
led_n_pos(xyz)                   m           3n+2:3n+4
.............camera parameters................
cam1_pos(xyz)                   m           3n+5:3n+7
cam1_ori(quaternion)                        3n+8:3n+11
cam1_k                          pixel       3n+12
..
cam_m_pos(xyz)                   m           3n+5+8m:3n+7+8m
cam_m_ori(quaternion)                        3n+8+8m:3n+11+8m
cam_m_k                          pixel       3n+12+8m

*/

#define LED_N 2
#define CAM_N 1

#define X_DIM 10
#define U_DIM 6

#define Z_N_CAM (CAM_N * LED_N)

#define C_DIM (2 + 3 * LED_N + 8 * CAM_N)

// #define Vout Ekf::feedV
// #define Mout Ekf::feedM

void transition_function(Vector &res, const Vector &x, const Vector &u, const data_type dt, const data_type g)
{
    // intermediate variables
    data_type dax = u.data[3] * dt, day = u.data[4] * dt, daz = u.data[5] * dt;

    // integration of linear velocity
    res.data[0] = x.data[0] + x.data[3] * dt;
    res.data[1] = x.data[1] + x.data[4] * dt;
    res.data[2] = x.data[2] + x.data[5] * dt;

    // integration of linear acceleration
    res.data[3] = x.data[3] + (dax * x.data[6] * x.data[6] - 2 * daz * x.data[6] * x.data[8] + 2 * day * x.data[6] * x.data[9] + dax * x.data[7] * x.data[7] + 2 * day * x.data[7] * x.data[8] + 2 * daz * x.data[7] * x.data[9] - dax * x.data[8] * x.data[8] - dax * x.data[9] * x.data[9] + u.data[0]) * dt;
    res.data[4] = x.data[4] + (day * x.data[6] * x.data[6] + 2 * daz * x.data[6] * x.data[7] - 2 * dax * x.data[6] * x.data[9] - day * x.data[7] * x.data[7] + 2 * dax * x.data[7] * x.data[8] + day * x.data[8] * x.data[8] + 2 * daz * x.data[8] * x.data[9] - day * x.data[9] * x.data[9] + u.data[1]) * dt;
    res.data[5] = x.data[5] + (daz * x.data[6] * x.data[6] - 2 * day * x.data[6] * x.data[7] + 2 * dax * x.data[6] * x.data[8] - daz * x.data[7] * x.data[7] + 2 * dax * x.data[7] * x.data[9] - daz * x.data[8] * x.data[8] + 2 * day * x.data[8] * x.data[9] + daz * x.data[9] * x.data[9] + u.data[2] + g) * dt;

    // integration of quaternion
    res.data[6] = x.data[6] - (dt * (u.data[0] * x.data[7] + u.data[1] * x.data[8] + u.data[2] * x.data[9])) / 2;
    res.data[7] = x.data[7] + (dt * (u.data[0] * x.data[6] - u.data[1] * x.data[9] + u.data[2] * x.data[8])) / 2;
    res.data[8] = x.data[8] + (dt * (u.data[1] * x.data[6] + u.data[0] * x.data[9] - u.data[2] * x.data[6])) / 2;
    res.data[9] = x.data[9] + (dt * (u.data[1] * x.data[7] - u.data[0] * x.data[8] + u.data[2] * x.data[6])) / 2;

    // normalize quaternion
    data_type norm_inv = 1 / sqrt(res.data[6] * res.data[6] + res.data[7] * res.data[7] + res.data[8] * res.data[8] + res.data[9] * res.data[9]);
    res.data[6] *= norm_inv;
    res.data[7] *= norm_inv;
    res.data[8] *= norm_inv;
    res.data[9] *= norm_inv;
}

// // fonction de mesure modulaire pour chaque couple led/cam
// void project(Vector res, const Vector &led_pos, const Quaternion &drone_orien, const Vector &drone_pos, const Vector &cam_position, Quaternion &cam_orien, const data_type cam_k)
// {
//     rotate(res, drone_orien, led_pos);
//     vector::add(res, res, drone_pos); // position of led in world frame
//     vector::sub(res, res, cam_position);

//     cam_orien.conjugate();
//     rotate(res, cam_orien, res); // position of led in cam frame
//     cam_orien.conjugate();

//     res.data[0] = res.data[0] * cam_k / res.data[2];
//     res.data[1] = res.data[1] * cam_k / res.data[2]; // position of led in the image
// }

// // fonction de mesure modulaire pour chaque couple led/cam = led_i/cam_j
// void hlicj(const Vector &x, const Vector &c, const uint_fast8_t i, const uint_fast8_t j)
// {
//     Vector drone_pos;
//     Quaternion drone_orien;
//     drone_pos.size = 3;
//     drone_pos.data = x.data;
//     drone_orien.data = x.data + 6;

//     Vector led_pos;
//     led_pos.size = 3;
//     led_pos.data = c.data + 2 + 3 * i;
//     data_type cam_k = c.data[2 + 3 * LED_N + 8 * j];

//     Vector cam_pos;
//     Quaternion cam_orien;
//     cam_pos.size = 3;
//     cam_pos.data = c.data + 2 + 3 * LED_N + 8 * j;
//     cam_orien.data = c.data + 5 + 8 * j;

//     project(Vout, led_pos, drone_orien, drone_pos, cam_pos, cam_orien, cam_k);
// }

// void hl0c0(const Vector &x, const Vector &c)
// {
//     hlicj(x, c, 0, 0);
// }

// void hl0c1(const Vector &x, const Vector &c)
// {
//     hlicj(x, c, 0, 1);
// }

// // fonction de mesure de position fake pour tester l'ekf sans caméra
// void h_fake_position(const Vector &x, const Vector &c)
// {
//     // fake position measurement
//     Vout.data[0] = x.data[0];
//     Vout.data[1] = x.data[1];
//     Vout.data[2] = x.data[2];
// }

#define GYRO_VAR 0.001 // in rad²/s²
#define ACC_VAR 0.01   // in (m/s²)²
#define CAM_VAR 4      // in pixel²
#define FAKE_POS_VAR 1 // in m²

#define INIT_POS_VAR 1e-1 // in m²
#define INIT_VEL_VAR 1e-3 // in (m/s)²
#define INIT_ATT_VAR 1e-6 // in rad² (more or less)

void Dasta::configureStateEstimate()
{
    // configure the ekf
    estimator.ekf = Ekf(X_DIM);
    estimator.ekf.x.fill(0);
    estimator.ekf.x.data[6] = 1; // qw

    estimator.ekf.P.fill(0);
    estimator.ekf.P(0, 0) = INIT_POS_VAR;
    estimator.ekf.P(1, 1) = INIT_POS_VAR;
    estimator.ekf.P(2, 2) = INIT_POS_VAR;
    estimator.ekf.P(3, 3) = INIT_VEL_VAR;
    estimator.ekf.P(4, 4) = INIT_VEL_VAR;
    estimator.ekf.P(5, 5) = INIT_VEL_VAR;
    estimator.ekf.P(6, 6) = INIT_ATT_VAR;
    estimator.ekf.P(7, 7) = INIT_ATT_VAR;
    estimator.ekf.P(8, 8) = INIT_ATT_VAR;
    estimator.ekf.P(9, 9) = INIT_ATT_VAR;
    estimator.gravity = 9.81;

    // link the pointers
    estimator.position.data = estimator.ekf.x.data;
    estimator.velocity.data = estimator.ekf.x.data + 3;
    estimator.orientation.data = estimator.ekf.x.data + 6;

    estimator.command.alloc(U_DIM);
    delete[] sensors.imu.gyro.data, sensors.imu.acc.data;
    sensors.imu.gyro.data = estimator.command.data;
    sensors.imu.acc.data = estimator.command.data + 3;

    estimator.command_cov.alloc(U_DIM);
    estimator.command_cov.fill(0);
    estimator.command_cov(0, 0) = GYRO_VAR;
    estimator.command_cov(1, 1) = GYRO_VAR;
    estimator.command_cov(2, 2) = GYRO_VAR;
    estimator.command_cov(3, 3) = ACC_VAR;
    estimator.command_cov(4, 4) = ACC_VAR;
    estimator.command_cov(5, 5) = ACC_VAR;

    sensors.cam = new Cam[CAM_N];
    for (uint_fast8_t i = 0; i < CAM_N; i++)
    {
        sensors.cam[i].led_predictions = new Vector[LED_N];
        sensors.cam[i].led_predictions_cov = new SymMatrix[LED_N];
        sensors.cam[i].led_predictions_cov_inv = new SymMatrix[LED_N];
        sensors.cam[i].led_predictions_jac = new Matrix[LED_N];
        sensors.cam[i].noise_cov.alloc(2);
        sensors.cam[i].noise_cov.fill(0);
        sensors.cam[i].noise_cov(0, 0) = CAM_VAR;
        sensors.cam[i].noise_cov(1, 1) = CAM_VAR;
        for (uint_fast8_t j = 0; j < LED_N; j++)
        {
            sensors.cam[i].led_predictions[j].alloc(2);
            sensors.cam[i].led_predictions_cov[j].alloc(2);
            sensors.cam[i].led_predictions_cov_inv[j].alloc(2);
            sensors.cam[i].led_predictions_jac[j].alloc(2, X_DIM);
        }
    }
}

void Dasta::runStateEstimate()
{
    if (estimator.running)
    {
        if (sensors.imu.fresh_data)
        {
            data_type dt = sensors.imu.getTime() - estimator.last_time_proprio;
            estimator.last_time_proprio = sensors.imu.getTime();
            auto f = [&dt, this](Vector &res, const Vector &x, const Vector &u)
            { transition_function(res, x, u, dt, this->estimator.gravity); };

            estimator.ekf.predict(f, estimator.command, estimator.command_cov);

            sensors.imu.fresh_data = false;
        }
        for (uint_fast8_t i = 0; i < CAM_N; i++)
        {
            for (uint_fast8_t j = 0; j < LED_N; j++)
            {
                // predict the position of the leds in the camera frame
                auto h = [&i, &j, this](const Vector &res, const Vector &x)
                { 
                    Vector drone_pos;
                    Quaternion drone_orien;
                    drone_pos.size = 3;
                    drone_pos.data = x.data;
                    drone_orien.data = x.data + 6;
                    sensors.cam[i].project(res, actuators.led[j].position, drone_orien, drone_pos);};
                estimator.ekf.predictMeasurment(h, sensors.cam[i].led_predictions[j], sensors.cam[i].noise_cov, sensors.cam[i].led_predictions_cov[j], &sensors.cam[i].led_predictions_jac[j], &sensors.cam[i].led_predictions_cov_inv[j]);
            }
            
            for (uint_fast8_t k = 0; k < sensors.cam[i].measurments_num; k++)
            {
                // predict the position of the leds in the camera frame

            }
        }
    }
}