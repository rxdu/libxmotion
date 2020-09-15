/* 
 * fxas21002c.hpp
 * 
 * Created on: Jan 12, 2018 23:56
 * Description: FXAS21002C linux driver
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef FXAS21002C_HPP
#define FXAS21002C_HPP

#include "imu/i2c.hpp"

namespace librav
{

// FXAS21002C I2C address
#define FXAS21002C_SLAVE_ADDR 0x20

// FXAS21002C internal register addresses
#define FXAS21002C_STATUS 0x00
#define FXAS21002C_OUT_X_MSB 0x01
#define FXAS21002C_OUT_X_LSB 0x02
#define FXAS21002C_OUT_Y_MSB 0x03
#define FXAS21002C_OUT_Y_LSB 0x04
#define FXAS21002C_OUT_Z_MSB 0x05
#define FXAS21002C_OUT_Z_LSB 0x06
#define FXAS21002C_WHO_AM_I 0x0C  // 11010111   r
#define FXAS21002C_CTRL_REG0 0x0D // 00000000   r/w
#define FXAS21002C_CTRL_REG1 0x13 // 00000000   r/w
#define FXAS21002C_CTRL_REG2 0x14 // 00000000   r/w
#define FXAS21002C_CTRL_REG3 0x15 // 00000000   r/w

#define FXAS21002C_WHOAMI_VAL 0xD7

// number of bytes to be read from the FXOS8700CQ
#define FXAS21002C_READ_LEN 7 // status plus 3 channels = 7 bytes

struct FXAS21002_RawData
{
  int16_t x;
  int16_t y;
  int16_t z;
};

enum FXAS21002GyroRange
{
  GYRO_RANGE_250DPS = 250,
  GYRO_RANGE_500DPS = 500,
  GYRO_RANGE_1000DPS = 1000,
  GYRO_RANGE_2000DPS = 2000
};

class FXAS21002C
{
public:
  FXAS21002C(int32_t i2c_dev_id, int32_t i2c_addr = FXAS21002C_SLAVE_ADDR);
  FXAS21002C() = delete;
  ~FXAS21002C() = default;

  bool ConfigI2CBus();
  bool ConfigSensor(FXAS21002GyroRange range = GYRO_RANGE_500DPS);
  bool ReadGyroData(FXAS21002_RawData *gyro_data);
  double GetGyroScale();

private:
  int32_t i2c_device_id_;
  int32_t i2c_slave_address_;
  I2CBus i2c_bus_;

  FXAS21002GyroRange gyro_range_;
};
}

#endif /* FXAS21002C_HPP */
