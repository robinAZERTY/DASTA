#ifndef SENSOR_PREPROCESSING_HPP
#define SENSOR_PREPROCESSING_HPP

#include <MPU9250.h>
#include "matrix.hpp"
#include "Cam.hpp"

class SensorPreProcessing
{
    public:
    SensorPreProcessing();
    ~SensorPreProcessing();

    void init();
    void readSensors();
    void compensateIMU();
    MPU9250 imu = MPU9250(Wire, 0x68);
    Cam cam1, cam2;
    Vector acc, gyro, mag;

    Vector acc_bias_co, gyro_bias_co, mag_bias_co;    // bias vector (3x1)
    Matrix acc_scale_co, gyro_scale_co, mag_scale_co; // scale matrix (3x3)

    bool imu_compensated = false;
    Vector tmp;
};
#endif // SENSOR_PREPROCESSING_HPP