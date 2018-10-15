/* 
 * fxos8700cq.hpp
 * 
 * Created on: Jan 12, 2018 21:18
 * Description: FXOS8700CQ linux driver
 *  Partially reused example code from FXOS8700CQ datasheet
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef FXOS8700CQ_HPP
#define FXOS8700CQ_HPP

#include <cstdint>

#include "i2c-bus/i2c.hpp"

namespace librav
{

// FXOS8700CQ I2C address
#define FXOS8700CQ_SLAVE_ADDR 0x1E // with pins SA0=0, SA1=0

// FXOS8700CQ internal register addresses
#define FXOS8700CQ_STATUS 0x00
#define FXOS8700CQ_WHOAMI 0x0D
#define FXOS8700CQ_XYZ_DATA_CFG 0x0E
#define FXOS8700CQ_CTRL_REG1 0x2A
#define FXOS8700CQ_M_CTRL_REG1 0x5B
#define FXOS8700CQ_M_CTRL_REG2 0x5C
#define FXOS8700CQ_WHOAMI_VAL 0xC7

// number of bytes to be read from the FXOS8700CQ
#define FXOS8700CQ_READ_LEN 13 // status plus 6 channels = 13 bytes

struct FXOS8700_RawData
{
    int16_t x;
    int16_t y;
    int16_t z;
};

// OPTIONAL SPEED SETTINGS
enum FXOS8700AccelRange
{
    ACCEL_RANGE_2G = 0x00,
    ACCEL_RANGE_4G = 0x01,
    ACCEL_RANGE_8G = 0x02
};

class FXOS8700CQ
{
  public:
    FXOS8700CQ(int32_t i2c_dev_id, int32_t i2c_addr = FXOS8700CQ_SLAVE_ADDR);
    FXOS8700CQ() = delete;
    ~FXOS8700CQ() = default;

    bool ConfigI2CBus();
    bool ConfigSensor(FXOS8700AccelRange range = ACCEL_RANGE_2G);
    bool ReadAccelMagnData(FXOS8700_RawData *acc_data, FXOS8700_RawData *mag_data);
    double GetAccelScale();
    double GetMagnScale();

  private:
    int32_t i2c_device_id_;
    int32_t i2c_slave_address_;
    I2CBus i2c_bus_;

    const double gravity_const = 9.80503;
    FXOS8700AccelRange accel_range_;
};
}

#endif /* FXOS8700CQ_HPP */
