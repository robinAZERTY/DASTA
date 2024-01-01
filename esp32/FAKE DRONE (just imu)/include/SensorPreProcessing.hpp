#ifndef SENSOR_PREPROCESSING_HPP
#define SENSOR_PREPROCESSING_HPP

#include <MPU9250.h>
#include "matrix.hpp"

class SensorPreProcessing
{
    unsigned long long time; // time in milliseconds
public:
    SensorPreProcessing();
    ~SensorPreProcessing();

    void init();
    void readSensors();
    void compensateIMU();
    const unsigned long long &getTime() { return time; };
    static MPU9250 imu;
    Vector acc, gyro, mag;

    Vector acc_bias, gyro_bias, mag_bias;    // bias vector (3x1)
    Matrix acc_scale, gyro_scale, mag_scale; // scale matrix (3x3)

    bool imu_compensated = false;
    Vector tmp = Vector(3);
};
#endif // SENSOR_PREPROCESSING_HPP