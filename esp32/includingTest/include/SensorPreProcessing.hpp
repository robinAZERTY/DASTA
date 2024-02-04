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

    MPU9250 imu;
    Cam *cam;
};
#endif // SENSOR_PREPROCESSING_HPP