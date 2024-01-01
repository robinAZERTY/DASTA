#include "SensorPreProcessing.hpp"

MPU9250 SensorPreProcessing::imu(Wire, 0x68);

SensorPreProcessing::SensorPreProcessing()
{
    acc.size = 3;
    gyro.size = 3;
    mag.size = 3;
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
    //override default compensation parameters
    // imu.setAccelCalX(0.0, 1.0);
    // imu.setAccelCalY(0.0, 1.0);
    // imu.setAccelCalZ(0.0, 1.0);
    // imu.setGyroBiasX_rads(0.0);
    // imu.setGyroBiasY_rads(0.0);
    // imu.setGyroBiasZ_rads(0.0);
    // imu.setMagCalX(0.0, 1.0);
    // imu.setMagCalY(0.0, 1.0);
    // imu.setMagCalZ(0.0, 1.0);



    //check if the Measurment vectors are cerrectly set

    if (acc.data == NULL || gyro.data == NULL || mag.data == NULL ||
        acc.size != 3 || gyro.size != 3 || mag.size != 3)
    {
        Serial.println("Measurment vectors are not set correctly!");
        while (1)
            ;
    }

    //check if all the compensation parameters are correctly set
    if (acc_bias.data == NULL || gyro_bias.data == NULL || mag_bias.data == NULL ||
        acc_scale.data == NULL || gyro_scale.data == NULL || mag_scale.data == NULL
        || acc_bias.size != 3 || gyro_bias.size != 3 || mag_bias.size != 3
        || acc_scale.size != 9 || gyro_scale.size != 9 || mag_scale.size != 9)
    {
        Serial.println("Compensation parameters are not set correctly!");
        while (1)
            ;
    }
}

void SensorPreProcessing::readSensors()
{
    imu.readSensor();
    acc(0) = imu.getAccelX_mss();
    acc(1) = imu.getAccelY_mss();
    acc(2) = imu.getAccelZ_mss();

    gyro(0) = imu.getGyroX_rads();
    gyro(1) = imu.getGyroY_rads();
    gyro(2) = imu.getGyroZ_rads();

    mag(0) = imu.getMagX_uT();
    mag(1) = imu.getMagY_uT();
    mag(2) = imu.getMagZ_uT();
    
    imu_compensated = false;
    time = millis();
}

void SensorPreProcessing::compensateIMU()
{
    if (imu_compensated)
        return;


    // data_type tmp[3];
    // Vector tmp_v;
    // tmp_v.data = tmp;
    // tmp_v.size = 3;

    //compensate accelerometer
    sub(acc, acc, acc_bias);
    mul(tmp, acc_scale, acc);
    cd(acc, tmp);

    //compensate gyroscope
    sub(gyro, gyro, gyro_bias);
    mul(tmp, gyro_scale, gyro);
    cd(gyro, tmp);

    //compensate magnetometer
    sub(mag, mag, mag_bias);
    mul(tmp, mag_scale, mag);
    cd(mag, tmp);

    imu_compensated = true;
}