#include "SensorPreProcessing.hpp"

SensorPreProcessing::SensorPreProcessing()
{
    acc.alloc(3);
    gyro.alloc(3);

    acc_raw.alloc(3);
    gyro_raw.alloc(3);

    acc_bias_co.alloc(3);
    gyro_bias_co.alloc(3);

    acc_bias_co.fill(0.0);
    gyro_bias_co.fill(0.0);

    acc_scale_co.alloc(3, 3);
    gyro_scale_co.alloc(3, 3);

    acc_scale_co.fill(0);
    gyro_scale_co.fill(0);

    tmp.alloc(3);
}

SensorPreProcessing::~SensorPreProcessing()
{
}

void SensorPreProcessing::init()
{
    int status = imu.begin();
    while (status < 0)
    {
        Serial.println("Failed to communicate with IMU!");
        Serial.println("error code: " + String(status));
        delay(1000);
    }

    // check if the Measurment vectors are correctly set
    if (acc.data == NULL || gyro.data == NULL ||
        acc.size != 3 || gyro.size != 3)
    {
        Serial.println("Measurment vectors are not set correctly!");
        while (1)
            ;
    }

    // check if all the compensation parameters are correctly set
    if (acc_bias_co.data == NULL || gyro_bias_co.data == NULL || acc_scale_co.data == NULL || gyro_scale_co.data == NULL || acc_bias_co.size != 3 || gyro_bias_co.size != 3 || acc_scale_co.size != 9 || gyro_scale_co.size != 9)
    {
        Serial.println("Compensation parameters are not set correctly!");
        while (1)
            ;
    }

    LiPo.init();
}

bool SensorPreProcessing::readSensors(const int64_t &now)
{
    if(imu.readSensor(now)<0)
        return false;

    acc.data[0] = imu.getAccelX_mss();
    acc.data[1] = imu.getAccelY_mss();
    acc.data[2] = imu.getAccelZ_mss();

    gyro.data[0] = imu.getGyroX_rads();
    gyro.data[1] = imu.getGyroY_rads();
    gyro.data[2] = imu.getGyroZ_rads();

    cd(acc_raw, acc);
    cd(gyro_raw, gyro);

    imu_compensated = false;
    return true;
}



void SensorPreProcessing::compensateIMU()
{
    if (imu_compensated)
        return;

    if (eq(acc_scale_co, 0) || eq(gyro_scale_co, 0))
       return; 

    calibrate = true;
    // compensate accelerometer
    add(acc, acc, acc_bias_co);
    mul(tmp, acc_scale_co, acc);
    cd(acc, tmp);

    // compensate gyroscope
    add(gyro, gyro, gyro_bias_co);

    mul(tmp, gyro_scale_co, gyro);
    cd(gyro, tmp);

    imu_compensated = true;
}