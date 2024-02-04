#include "SensorPreProcessing.hpp"

SensorPreProcessing::SensorPreProcessing()
{
}

SensorPreProcessing::~SensorPreProcessing()
{
}

void SensorPreProcessing::init()
{
    imu.set(Wire, 0x68);

    if (imu.begin()<0)
    {
        Serial.println("Failed to communicate with IMU!");
        while (1)
            ;
    }
}

void SensorPreProcessing::readSensors()
{
    imu.readSensor();
}
