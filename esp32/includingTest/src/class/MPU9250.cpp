/*
  MPU9250.cpp
  Brian R Taylor
  brian.taylor@bolderflight.com

  Copyright (c) 2017 Bolder Flight Systems

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "Arduino.h"
#include "MPU9250.h"

MPU9250::MPU9250()
{
  acc.alloc(3);
  gyro.alloc(3);
  mag.alloc(3);
  acc_bias_co.alloc(3);
  gyro_bias_co.alloc(3);
  mag_bias_co.alloc(3);
  acc_scale_co.alloc(3, 3);
  gyro_scale_co.alloc(3, 3);
  mag_scale_co.alloc(3, 3);
  tmp.alloc(3);

  acc_bias_co.fill(0);
  gyro_bias_co.fill(0);
  mag_bias_co.fill(0);

  acc_scale_co.set_eye();
  gyro_scale_co.set_eye();
  mag_scale_co.set_eye();
}

void MPU9250::set(TwoWire &bus, uint8_t address)
{
  _i2c = &bus;        // I2C bus
  _address = address; // I2C address
  _useSPI = false;    // set to use I2C
}

void MPU9250::set(SPIClass &bus, uint8_t csPin)
{
  _spi = &bus;    // SPI bus
  _csPin = csPin; // chip select pin
  _useSPI = true; // set to use SPI
}

/* starts communication with the MPU-9250 */
int MPU9250::begin()
{
  if (_useSPI)
  { // using SPI for communication
    // use low speed SPI for register setting
    _useSPIHS = false;
    // setting CS pin to output
    pinMode(_csPin, OUTPUT);
    // setting CS pin high
    digitalWrite(_csPin, HIGH);
    // begin SPI communication
    _spi->begin();
  }
  else
  { // using I2C for communication
    // starting the I2C bus
    _i2c->begin();
    // setting the I2C clock
    _i2c->setClock(_i2cRate);
  }
  // select clock source to gyro
  if (writeRegister(PWR_MGMNT_1, CLOCK_SEL_PLL) < 0)
  {
    return -1;
  }
  // enable I2C master mode
  if (writeRegister(USER_CTRL, I2C_MST_EN) < 0)
  {
    return -2;
  }
  // set the I2C bus speed to 400 kHz
  if (writeRegister(I2C_MST_CTRL, I2C_MST_CLK) < 0)
  {
    return -3;
  }
  // set AK8963 to Power Down
  writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);
  // reset the MPU9250
  writeRegister(PWR_MGMNT_1, PWR_RESET);
  // wait for MPU-9250 to come back up
  delay(1);
  // reset the AK8963
  writeAK8963Register(AK8963_CNTL2, AK8963_RESET);
  // select clock source to gyro
  if (writeRegister(PWR_MGMNT_1, CLOCK_SEL_PLL) < 0)
  {
    return -4;
  }
  // check the WHO AM I byte, expected value is 0x71 (decimal 113) or 0x73 (decimal 115)
  if ((whoAmI() != 113) && (whoAmI() != 115))
  {
    return -5;
  }
  // enable accelerometer and gyro
  if (writeRegister(PWR_MGMNT_2, SEN_ENABLE) < 0)
  {
    return -6;
  }
  // setting accel range to 16G as default
  if (writeRegister(ACCEL_CONFIG, ACCEL_FS_SEL_16G) < 0)
  {
    return -7;
  }
  _accelScale = G * 16.0f / 32767.5f; // setting the accel scale to 16G
  _accelRange = ACCEL_RANGE_16G;
  // setting the gyro range to 2000DPS as default
  if (writeRegister(GYRO_CONFIG, GYRO_FS_SEL_2000DPS) < 0)
  {
    return -8;
  }
  _gyroScale = 2000.0f / 32767.5f * _d2r; // setting the gyro scale to 2000DPS
  _gyroRange = GYRO_RANGE_2000DPS;
  // setting bandwidth to 184Hz as default
  if (writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_184) < 0)
  {
    return -9;
  }
  if (writeRegister(CONFIG, GYRO_DLPF_184) < 0)
  { // setting gyro bandwidth to 184Hz
    return -10;
  }
  _bandwidth = DLPF_BANDWIDTH_184HZ;
  // setting the sample rate divider to 0 as default
  if (writeRegister(SMPDIV, 0x00) < 0)
  {
    return -11;
  }
  _srd = 0;
  // enable I2C master mode
  if (writeRegister(USER_CTRL, I2C_MST_EN) < 0)
  {
    return -12;
  }
  // set the I2C bus speed to 400 kHz
  if (writeRegister(I2C_MST_CTRL, I2C_MST_CLK) < 0)
  {
    return -13;
  }
  // check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
  if (whoAmIAK8963() != 72)
  {
    return -14;
  }
  /* get the magnetometer calibration */
  // set AK8963 to Power Down
  if (writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN) < 0)
  {
    return -15;
  }
  delay(100); // long wait between AK8963 mode changes
  // set AK8963 to FUSE ROM access
  if (writeAK8963Register(AK8963_CNTL1, AK8963_FUSE_ROM) < 0)
  {
    return -16;
  }
  delay(100); // long wait between AK8963 mode changes
  // read the AK8963 ASA registers and compute magnetometer scale factors
  readAK8963Registers(AK8963_ASA, 3, _buffer);
  _magScaleX = ((((float)_buffer[0]) - 128.0f) / (256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
  _magScaleY = ((((float)_buffer[1]) - 128.0f) / (256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
  _magScaleZ = ((((float)_buffer[2]) - 128.0f) / (256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
  // set AK8963 to Power Down
  if (writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN) < 0)
  {
    return -17;
  }
  delay(100); // long wait between AK8963 mode changes
  // set AK8963 to 16 bit resolution, 100 Hz update rate
  if (writeAK8963Register(AK8963_CNTL1, AK8963_CNT_MEAS2) < 0)
  {
    return -18;
  }
  delay(100); // long wait between AK8963 mode changes
  // select clock source to gyro
  if (writeRegister(PWR_MGMNT_1, CLOCK_SEL_PLL) < 0)
  {
    return -19;
  }
  // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
  readAK8963Registers(AK8963_HXL, 7, _buffer);
  // estimate gyro bias
  // if (calibrateGyro() < 0) {
  //   return -20;
  // }
  // successful init, return 1
  return 1;
}

/* sets the accelerometer full scale range to values other than default */
int MPU9250::setAccelRange(AccelRange range)
{
  // use low speed SPI for register setting
  _useSPIHS = false;
  switch (range)
  {
  case ACCEL_RANGE_2G:
  {
    // setting the accel range to 2G
    if (writeRegister(ACCEL_CONFIG, ACCEL_FS_SEL_2G) < 0)
    {
      return -1;
    }
    _accelScale = G * 2.0f / 32767.5f; // setting the accel scale to 2G
    break;
  }
  case ACCEL_RANGE_4G:
  {
    // setting the accel range to 4G
    if (writeRegister(ACCEL_CONFIG, ACCEL_FS_SEL_4G) < 0)
    {
      return -1;
    }
    _accelScale = G * 4.0f / 32767.5f; // setting the accel scale to 4G
    break;
  }
  case ACCEL_RANGE_8G:
  {
    // setting the accel range to 8G
    if (writeRegister(ACCEL_CONFIG, ACCEL_FS_SEL_8G) < 0)
    {
      return -1;
    }
    _accelScale = G * 8.0f / 32767.5f; // setting the accel scale to 8G
    break;
  }
  case ACCEL_RANGE_16G:
  {
    // setting the accel range to 16G
    if (writeRegister(ACCEL_CONFIG, ACCEL_FS_SEL_16G) < 0)
    {
      return -1;
    }
    _accelScale = G * 16.0f / 32767.5f; // setting the accel scale to 16G
    break;
  }
  }
  _accelRange = range;
  return 1;
}

/* sets the gyro full scale range to values other than default */
int MPU9250::setGyroRange(GyroRange range)
{
  // use low speed SPI for register setting
  _useSPIHS = false;
  switch (range)
  {
  case GYRO_RANGE_250DPS:
  {
    // setting the gyro range to 250DPS
    if (writeRegister(GYRO_CONFIG, GYRO_FS_SEL_250DPS) < 0)
    {
      return -1;
    }
    _gyroScale = 250.0f / 32767.5f * _d2r; // setting the gyro scale to 250DPS
    break;
  }
  case GYRO_RANGE_500DPS:
  {
    // setting the gyro range to 500DPS
    if (writeRegister(GYRO_CONFIG, GYRO_FS_SEL_500DPS) < 0)
    {
      return -1;
    }
    _gyroScale = 500.0f / 32767.5f * _d2r; // setting the gyro scale to 500DPS
    break;
  }
  case GYRO_RANGE_1000DPS:
  {
    // setting the gyro range to 1000DPS
    if (writeRegister(GYRO_CONFIG, GYRO_FS_SEL_1000DPS) < 0)
    {
      return -1;
    }
    _gyroScale = 1000.0f / 32767.5f * _d2r; // setting the gyro scale to 1000DPS
    break;
  }
  case GYRO_RANGE_2000DPS:
  {
    // setting the gyro range to 2000DPS
    if (writeRegister(GYRO_CONFIG, GYRO_FS_SEL_2000DPS) < 0)
    {
      return -1;
    }
    _gyroScale = 2000.0f / 32767.5f * _d2r; // setting the gyro scale to 2000DPS
    break;
  }
  }
  _gyroRange = range;
  return 1;
}

/* sets the DLPF bandwidth to values other than default */
int MPU9250::setDlpfBandwidth(DlpfBandwidth bandwidth)
{
  // use low speed SPI for register setting
  _useSPIHS = false;
  switch (bandwidth)
  {
  case DLPF_BANDWIDTH_184HZ:
  {
    if (writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_184) < 0)
    { // setting accel bandwidth to 184Hz
      return -1;
    }
    if (writeRegister(CONFIG, GYRO_DLPF_184) < 0)
    { // setting gyro bandwidth to 184Hz
      return -2;
    }
    break;
  }
  case DLPF_BANDWIDTH_92HZ:
  {
    if (writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_92) < 0)
    { // setting accel bandwidth to 92Hz
      return -1;
    }
    if (writeRegister(CONFIG, GYRO_DLPF_92) < 0)
    { // setting gyro bandwidth to 92Hz
      return -2;
    }
    break;
  }
  case DLPF_BANDWIDTH_41HZ:
  {
    if (writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_41) < 0)
    { // setting accel bandwidth to 41Hz
      return -1;
    }
    if (writeRegister(CONFIG, GYRO_DLPF_41) < 0)
    { // setting gyro bandwidth to 41Hz
      return -2;
    }
    break;
  }
  case DLPF_BANDWIDTH_20HZ:
  {
    if (writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_20) < 0)
    { // setting accel bandwidth to 20Hz
      return -1;
    }
    if (writeRegister(CONFIG, GYRO_DLPF_20) < 0)
    { // setting gyro bandwidth to 20Hz
      return -2;
    }
    break;
  }
  case DLPF_BANDWIDTH_10HZ:
  {
    if (writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_10) < 0)
    { // setting accel bandwidth to 10Hz
      return -1;
    }
    if (writeRegister(CONFIG, GYRO_DLPF_10) < 0)
    { // setting gyro bandwidth to 10Hz
      return -2;
    }
    break;
  }
  case DLPF_BANDWIDTH_5HZ:
  {
    if (writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_5) < 0)
    { // setting accel bandwidth to 5Hz
      return -1;
    }
    if (writeRegister(CONFIG, GYRO_DLPF_5) < 0)
    { // setting gyro bandwidth to 5Hz
      return -2;
    }
    break;
  }
  }
  _bandwidth = bandwidth;
  return 1;
}

/* sets the sample rate divider to values other than default */
int MPU9250::setSrd(uint8_t srd)
{
  // use low speed SPI for register setting
  _useSPIHS = false;
  /* setting the sample rate divider to 19 to facilitate setting up magnetometer */
  if (writeRegister(SMPDIV, 19) < 0)
  { // setting the sample rate divider
    return -1;
  }
  if (srd > 9)
  {
    // set AK8963 to Power Down
    if (writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN) < 0)
    {
      return -2;
    }
    delay(100); // long wait between AK8963 mode changes
    // set AK8963 to 16 bit resolution, 8 Hz update rate
    if (writeAK8963Register(AK8963_CNTL1, AK8963_CNT_MEAS1) < 0)
    {
      return -3;
    }
    delay(100); // long wait between AK8963 mode changes
    // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
    readAK8963Registers(AK8963_HXL, 7, _buffer);
  }
  else
  {
    // set AK8963 to Power Down
    if (writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN) < 0)
    {
      return -2;
    }
    delay(100); // long wait between AK8963 mode changes
    // set AK8963 to 16 bit resolution, 100 Hz update rate
    if (writeAK8963Register(AK8963_CNTL1, AK8963_CNT_MEAS2) < 0)
    {
      return -3;
    }
    delay(100); // long wait between AK8963 mode changes
    // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
    readAK8963Registers(AK8963_HXL, 7, _buffer);
  }
  /* setting the sample rate divider */
  if (writeRegister(SMPDIV, srd) < 0)
  { // setting the sample rate divider
    return -4;
  }
  _srd = srd;
  return 1;
}

/* enables the data ready interrupt */
int MPU9250::enableDataReadyInterrupt()
{
  // use low speed SPI for register setting
  _useSPIHS = false;
  /* setting the interrupt */
  if (writeRegister(INT_PIN_CFG, INT_PULSE_50US) < 0)
  { // setup interrupt, 50 us pulse
    return -1;
  }
  if (writeRegister(INT_ENABLE, INT_RAW_RDY_EN) < 0)
  { // set to data ready
    return -2;
  }
  return 1;
}

/* disables the data ready interrupt */
int MPU9250::disableDataReadyInterrupt()
{
  // use low speed SPI for register setting
  _useSPIHS = false;
  if (writeRegister(INT_ENABLE, INT_DISABLE) < 0)
  { // disable interrupt
    return -1;
  }
  return 1;
}

/* configures and enables wake on motion, low power mode */
int MPU9250::enableWakeOnMotion(float womThresh_mg, LpAccelOdr odr)
{
  // use low speed SPI for register setting
  _useSPIHS = false;
  // set AK8963 to Power Down
  writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);
  // reset the MPU9250
  writeRegister(PWR_MGMNT_1, PWR_RESET);
  // wait for MPU-9250 to come back up
  delay(1);
  if (writeRegister(PWR_MGMNT_1, 0x00) < 0)
  { // cycle 0, sleep 0, standby 0
    return -1;
  }
  if (writeRegister(PWR_MGMNT_2, DIS_GYRO) < 0)
  { // disable gyro measurements
    return -2;
  }
  if (writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_184) < 0)
  { // setting accel bandwidth to 184Hz
    return -3;
  }
  if (writeRegister(INT_ENABLE, INT_WOM_EN) < 0)
  { // enabling interrupt to wake on motion
    return -4;
  }
  if (writeRegister(MOT_DETECT_CTRL, (ACCEL_INTEL_EN | ACCEL_INTEL_MODE)) < 0)
  { // enabling accel hardware intelligence
    return -5;
  }
  _womThreshold = map(womThresh_mg, 0, 1020, 0, 255);
  if (writeRegister(WOM_THR, _womThreshold) < 0)
  { // setting wake on motion threshold
    return -6;
  }
  if (writeRegister(LP_ACCEL_ODR, (uint8_t)odr) < 0)
  { // set frequency of wakeup
    return -7;
  }
  if (writeRegister(PWR_MGMNT_1, PWR_CYCLE) < 0)
  { // switch to accel low power mode
    return -8;
  }
  return 1;
}

/* reads the most current data from MPU9250 and stores in buffer */
int MPU9250::readSensor()
{
  _useSPIHS = true; // use the high speed SPI for data readout
  // grab the data from the MPU9250
  if (readRegisters(ACCEL_OUT, 21, _buffer) < 0)
  {
    return -1;
  }
  _time = millis();
  // combine into 16 bit values
  _axcounts = (((int16_t)_buffer[0]) << 8) | _buffer[1];
  _aycounts = (((int16_t)_buffer[2]) << 8) | _buffer[3];
  _azcounts = (((int16_t)_buffer[4]) << 8) | _buffer[5];
  _tcounts = (((int16_t)_buffer[6]) << 8) | _buffer[7];
  _gxcounts = (((int16_t)_buffer[8]) << 8) | _buffer[9];
  _gycounts = (((int16_t)_buffer[10]) << 8) | _buffer[11];
  _gzcounts = (((int16_t)_buffer[12]) << 8) | _buffer[13];
  _hxcounts = (((int16_t)_buffer[15]) << 8) | _buffer[14];
  _hycounts = (((int16_t)_buffer[17]) << 8) | _buffer[16];
  _hzcounts = (((int16_t)_buffer[19]) << 8) | _buffer[18];
  // transform and convert to float values
  acc.data[0] = (float)(tX[0] * _axcounts + tX[1] * _aycounts + tX[2] * _azcounts) * _accelScale;
  acc.data[1] = (float)(tY[0] * _axcounts + tY[1] * _aycounts + tY[2] * _azcounts) * _accelScale;
  acc.data[2] = (float)(tZ[0] * _axcounts + tZ[1] * _aycounts + tZ[2] * _azcounts) * _accelScale;
  gyro.data[0] = (float)(tX[0] * _gxcounts + tX[1] * _gycounts + tX[2] * _gzcounts) * _gyroScale;
  gyro.data[1] = (float)(tY[0] * _gxcounts + tY[1] * _gycounts + tY[2] * _gzcounts) * _gyroScale;
  gyro.data[2] = (float)(tZ[0] * _gxcounts + tZ[1] * _gycounts + tZ[2] * _gzcounts) * _gyroScale;
  mag.data[0] = (float)(_hxcounts)*_magScaleX;
  mag.data[1] = (float)(_hycounts)*_magScaleY;
  mag.data[2] = (float)(_hzcounts)*_magScaleZ;
  _t = ((((float)_tcounts) - _tempOffset) / _tempScale) + _tempOffset;

  compensated = false;

  return 1;
}

void MPU9250::compensate()
{
  if (compensated)
    return;

  // compensate accelerometer
  vector::add(acc, acc, acc_bias_co);
  matrix::mul(tmp, acc_scale_co, acc);
  vector::cd(acc, tmp);

  // compensate gyroscope
  vector::add(gyro, gyro, gyro_bias_co);
  matrix::mul(tmp, gyro_scale_co, gyro);
  vector::cd(gyro, tmp);

  // compensate magnetometer
  vector::add(mag, mag, mag_bias_co);
  matrix::mul(tmp, mag_scale_co, mag);
  vector::cd(mag, tmp);

  compensated = true;
  fresh_data = true;
}

/* returns the die temperature, C */
float MPU9250::getTemperature_C()
{
  return _t;
}

/* writes a byte to MPU9250 register given a register address and data */
int MPU9250::writeRegister(uint8_t subAddress, uint8_t data)
{
  /* write data to device */
  if (_useSPI)
  {
    _spi->beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
    digitalWrite(_csPin, LOW);                                              // select the MPU9250 chip
    _spi->transfer(subAddress);                                             // write the register address
    _spi->transfer(data);                                                   // write the data
    digitalWrite(_csPin, HIGH);                                             // deselect the MPU9250 chip
    _spi->endTransaction();                                                 // end the transaction
  }
  else
  {
    _i2c->beginTransmission(_address); // open the device
    _i2c->write(subAddress);           // write the register address
    _i2c->write(data);                 // write the data
    _i2c->endTransmission();
  }

  delay(10);

  /* read back the register */
  readRegisters(subAddress, 1, _buffer);
  /* check the read back register against the written register */
  if (_buffer[0] == data)
  {
    return 1;
  }
  else
  {
    return -1;
  }
}

/* reads registers from MPU9250 given a starting register address, number of bytes, and a pointer to store data */
int MPU9250::readRegisters(uint8_t subAddress, uint8_t count, uint8_t *dest)
{
  if (_useSPI)
  {
    // begin the transaction
    if (_useSPIHS)
    {
      _spi->beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE3));
    }
    else
    {
      _spi->beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
    }
    digitalWrite(_csPin, LOW);             // select the MPU9250 chip
    _spi->transfer(subAddress | SPI_READ); // specify the starting register address
    for (uint8_t i = 0; i < count; i++)
    {
      dest[i] = _spi->transfer(0x00); // read the data
    }
    digitalWrite(_csPin, HIGH); // deselect the MPU9250 chip
    _spi->endTransaction();     // end the transaction
    return 1;
  }
  else
  {
    _i2c->beginTransmission(_address); // open the device
    _i2c->write(subAddress);           // specify the starting register address
    _i2c->endTransmission(false);
    _numBytes = _i2c->requestFrom(_address, count); // specify the number of bytes to receive
    if (_numBytes == count)
    {
      for (uint8_t i = 0; i < count; i++)
      {
        dest[i] = _i2c->read();
      }
      return 1;
    }
    else
    {
      return -1;
    }
  }
}

/* writes a register to the AK8963 given a register address and data */
int MPU9250::writeAK8963Register(uint8_t subAddress, uint8_t data)
{
  // set slave 0 to the AK8963 and set for write
  if (writeRegister(I2C_SLV0_ADDR, AK8963_I2C_ADDR) < 0)
  {
    return -1;
  }
  // set the register to the desired AK8963 sub address
  if (writeRegister(I2C_SLV0_REG, subAddress) < 0)
  {
    return -2;
  }
  // store the data for write
  if (writeRegister(I2C_SLV0_DO, data) < 0)
  {
    return -3;
  }
  // enable I2C and send 1 byte
  if (writeRegister(I2C_SLV0_CTRL, I2C_SLV0_EN | (uint8_t)1) < 0)
  {
    return -4;
  }
  // read the register and confirm
  if (readAK8963Registers(subAddress, 1, _buffer) < 0)
  {
    return -5;
  }
  if (_buffer[0] == data)
  {
    return 1;
  }
  else
  {
    return -6;
  }
}

/* reads registers from the AK8963 */
int MPU9250::readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t *dest)
{
  // set slave 0 to the AK8963 and set for read
  if (writeRegister(I2C_SLV0_ADDR, AK8963_I2C_ADDR | I2C_READ_FLAG) < 0)
  {
    return -1;
  }
  // set the register to the desired AK8963 sub address
  if (writeRegister(I2C_SLV0_REG, subAddress) < 0)
  {
    return -2;
  }
  // enable I2C and request the bytes
  if (writeRegister(I2C_SLV0_CTRL, I2C_SLV0_EN | count) < 0)
  {
    return -3;
  }
  delay(1); // takes some time for these registers to fill
            // read the bytes off the MPU9250 EXT_SENS_DATA registers
  _status = readRegisters(EXT_SENS_DATA_00, count, dest);
  return _status;
}

/* gets the MPU9250 WHO_AM_I register value, expected to be 0x71 */
int MPU9250::whoAmI()
{
  // read the WHO AM I register
  if (readRegisters(WHO_AM_I, 1, _buffer) < 0)
  {
    return -1;
  }
  // return the register value
  return _buffer[0];
}

/* gets the AK8963 WHO_AM_I register value, expected to be 0x48 */
int MPU9250::whoAmIAK8963()
{
  // read the WHO AM I register
  if (readAK8963Registers(AK8963_WHO_AM_I, 1, _buffer) < 0)
  {
    return -1;
  }
  // return the register value
  return _buffer[0];
}
