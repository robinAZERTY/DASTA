#ifndef SENSOR_PREPROCESSING_HPP
#define SENSOR_PREPROCESSING_HPP

#include <MPU6500.h>
#include "batteryTracker.hpp"
#include "matrix.hpp"
// #include "Cam.hpp"


class SensorPreProcessing
{
    public:
    SensorPreProcessing();
    ~SensorPreProcessing();

    void init();
    bool readSensors(const int64_t &now = esp_timer_get_time());
    void compensateIMU();
    BatteryTracker3s LiPo;
    MPU6500 imu = MPU6500(Wire, 0x68);
    // Cam cam1, cam2;
    Vector acc, gyro;
    Vector acc_raw, gyro_raw;

    Vector acc_bias_co, gyro_bias_co;    // bias vector (3x1)
    Matrix acc_scale_co, gyro_scale_co; // scale matrix (3x3)
    bool calibrate = false;
    bool imu_compensated = false;
    Vector tmp;
};
#endif // SENSOR_PREPROCESSING_HPP