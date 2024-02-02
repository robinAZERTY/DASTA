#include "SensorPreProcessing.hpp"

SensorPreProcessing::SensorPreProcessing()
{

    acc.size = 3;
    gyro.size = 3;
    mag.size = 3;

    acc_bias_co.alloc(3);
    gyro_bias_co.alloc(3);
    mag_bias_co.alloc(3);

    acc_scale_co.alloc(3, 3);
    gyro_scale_co.alloc(3, 3);
    mag_scale_co.alloc(3, 3);

    acc_bias_co.fill(0);
    gyro_bias_co.fill(0);
    mag_bias_co.fill(0);

    acc_scale_co.set_eye();
    gyro_scale_co.set_eye();
    mag_scale_co.set_eye();

    tmp.alloc(3);
}

SensorPreProcessing::~SensorPreProcessing()
{
}

void SensorPreProcessing::init()
{
    
    if (imu.begin()<0)
    {
        Serial.println("Failed to communicate with IMU!");
        while (1)
            ;
    }

    //check if the Measurment vectors are correctly set
    if (acc.data == NULL || gyro.data == NULL || mag.data == NULL ||
        acc.size != 3 || gyro.size != 3 || mag.size != 3)
    {
        Serial.println("Measurment vectors are not set correctly!");
        while (1)
            ;
    }

    //check if all the compensation parameters are correctly set
    if (acc_bias_co.data == NULL || gyro_bias_co.data == NULL || mag_bias_co.data == NULL ||
        acc_scale_co.data == NULL || gyro_scale_co.data == NULL || mag_scale_co.data == NULL
        || acc_bias_co.size != 3 || gyro_bias_co.size != 3 || mag_bias_co.size != 3
        || acc_scale_co.size != 9 || gyro_scale_co.size != 9 || mag_scale_co.size != 9)
    {
        Serial.println("Compensation parameters are not set correctly!");
        while (1)
            ;
    }
}

void SensorPreProcessing::readSensors()
{
    imu.readSensor();

    acc.data[0] = imu.getAccelX_mss();
    acc.data[1] = imu.getAccelY_mss();
    acc.data[2] = imu.getAccelZ_mss();

    gyro.data[0] = imu.getGyroX_rads();
    gyro.data[1] = imu.getGyroY_rads();
    gyro.data[2] = imu.getGyroZ_rads();

    mag.data[0] = imu.getMagX_uT();
    mag.data[1] = imu.getMagY_uT();
    mag.data[2] = imu.getMagZ_uT();
    
    imu_compensated = false;
    gyro_bias_compensated = false;

    if (gyro_bias_estimation_running)
    {
        gyro_bias_estimation_count++;
        vector::add(tmp, gyro_bias_co, gyro);
        vector::mul(tmp, tmp, -1.0 / gyro_bias_estimation_count);
        vector::add(gyro_bias_co, gyro_bias_co, tmp);
    }
}

void SensorPreProcessing::startGyroBiasEstimation()
{
    gyro_bias_estimation_count = 0;
    gyro_bias_estimation_running = true;
    vector::cd(gyro_bias_co, gyro);
    vector::mul(gyro_bias_co, gyro_bias_co, -1.0);
}

void SensorPreProcessing::compensateGyroBias()
{
    if (gyro_bias_compensated)
        return;

    //compensate gyroscope
    vector::add(gyro, gyro, gyro_bias_co);

    gyro_bias_compensated = true;
}

void SensorPreProcessing::compensateIMU()
{
    if (imu_compensated)
        return;

    //compensate accelerometer
    vector::add(acc, acc, acc_bias_co);
    matrix::mul(tmp, acc_scale_co, acc);
    vector::cd(acc, tmp);

    //compensate gyroscope
    if (!gyro_bias_compensated)
        vector::add(gyro, gyro, gyro_bias_co);

    matrix::mul(tmp, gyro_scale_co, gyro);
    vector::cd(gyro, tmp);

    //compensate magnetometer
    vector::add(mag, mag, mag_bias_co);
    matrix::mul(tmp, mag_scale_co, mag);
    vector::cd(mag, tmp);

    imu_compensated = true;
}