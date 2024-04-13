#include "Dasta.hpp"

// devider resister bridge for reading the battery voltage (3 cells)
// cell 1
#define R1 3280
#define R2 10030
#define PIN_CELL_1 32
// cell 1 and 2 in series
#define R3 17630
#define R4 10040
#define PIN_CELL_12 35
// cell 1, 2 and 3 in series
#define R5 39200
#define R6 9995
#define PIN_CELL_123 34

// power (the bigger cable)
#define R7 56015
#define R8 10000
#define PIN_POWER 33


#define INTERNAL_RES 0.02

//voltage samples
float BatteryTracker3s::charge_sample[4][2] = {{0.0,3.0},{0.2,3.7},{0.5,3.8},{1.0,4.2}};
uint8_t BatteryTracker3s::sample_size = 4;

void Dasta::configSensorPreProcessing()
{
    // allocate and link the matrices and vectors
    sensors.acc_bias_co.data = new data_type[3];
    sensors.gyro_bias_co.data = new data_type[3];

    sensors.acc_scale_co.data = new data_type[9];
    sensors.gyro_scale_co.data = new data_type[9];


    sensors.acc_bias_co.fill(0);
    sensors.gyro_bias_co.fill(0);

    sensors.acc_scale_co.set_eye();
    sensors.gyro_scale_co.set_eye();


    // (sensors.acc_bias_co = Vector(3)).fill(0);
    // (sensors.gyro_bias_co = Vector(3)).fill(0);
    // (sensors.mag_bias_co = Vector(3)).fill(0);

    // (sensors.acc_scale_co = Matrix(3, 3)).set_eye();
    // (sensors.gyro_scale_co = Matrix(3, 3)).set_eye();
    // (sensors.mag_scale_co = Matrix(3, 3)).set_eye();

    // sensors.gyro.data = estimator.ekf->u->data;
    // sensors.acc.data = estimator.ekf->u->data + 3;
    // sensors.acc.data = new data_type[3];
    // sensors.gyro.data = new data_type[3];
    sensors.LiPo._R1 = R1;
    sensors.LiPo._R2 = R2;
    sensors.LiPo._R3 = R3;
    sensors.LiPo._R4 = R4;
    sensors.LiPo._R5 = R5;
    sensors.LiPo._R6 = R6;
    sensors.LiPo._R7 = R7;
    sensors.LiPo._R8 = R8;
    sensors.LiPo.pinCell1 = PIN_CELL_1;
    sensors.LiPo.pinCell2 = PIN_CELL_12;
    sensors.LiPo.pinCell3 = PIN_CELL_123;
    sensors.LiPo.powerPin = PIN_POWER;
    sensors.LiPo.internal_resistance = INTERNAL_RES;
    sensors.LiPo.lpfc1.setTimeConstant(10);
    sensors.LiPo.lpfc2.setTimeConstant(10);
    sensors.LiPo.lpfc3.setTimeConstant(10);
    sensors.LiPo.lpfc4.setTimeConstant(10);
    
}