#ifndef SENSOR_PREPROCESSING_HPP
#define SENSOR_PREPROCESSING_HPP

#include <MPU6500.h>
#include "matrix.hpp"
// #include "Cam.hpp"

class SensorPreProcessing
{
    public:
    SensorPreProcessing();
    ~SensorPreProcessing();

    void init();
    void readSensors();
    void compensateIMU();
    void compensateGyroBias();

    void startGyroBiasEstimation();
    MPU6500 imu = MPU6500(Wire, 0x68);
    // Cam cam1, cam2;
    Vector acc, gyro;

    Vector acc_bias_co, gyro_bias_co;    // bias vector (3x1)
    Matrix acc_scale_co, gyro_scale_co; // scale matrix (3x3)

    bool imu_compensated = false;
    Vector tmp;

    bool gyro_bias_estimation_running = false;
    bool gyro_bias_compensated = false;
    unsigned long long gyro_bias_estimation_count = 0;
};
#endif // SENSOR_PREPROCESSING_HPP