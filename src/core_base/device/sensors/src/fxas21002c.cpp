/* 
 * fxas21002c.cpp
 * 
 * Created on: Jan 12, 2018 23:58
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "sensors/fxas21002c.hpp"

using namespace librav;

FXAS21002C::FXAS21002C(int32_t i2c_dev_id, int32_t i2c_addr) : i2c_device_id_(i2c_dev_id),
                                                               i2c_slave_address_(i2c_addr)
{
}

bool FXAS21002C::ConfigI2CBus()
{
    if (!i2c_bus_.Init(i2c_device_id_))
        return false;

    if (!i2c_bus_.SetSlaveAddress(i2c_slave_address_))
        return false;

    // read and check the FXAS21002C WHOAMI register
    uint8_t databyte;
    if (!i2c_bus_.ReadRegister(FXAS21002C_WHO_AM_I, &databyte))
        return false;

    if (databyte != FXAS21002C_WHOAMI_VAL)
        return false;

    return true;
}

bool FXAS21002C::ConfigSensor(FXAS21002GyroRange range)
{
    uint8_t databyte;

    // set gyro in standby mode
    databyte = 0x00;
    if (!i2c_bus_.WriteRegister(FXAS21002C_CTRL_REG1, databyte))
        return false;

    // set range
    switch (range)
    {
    case GYRO_RANGE_250DPS:
        databyte = 0x03;
        break;
    case GYRO_RANGE_500DPS:
        databyte = 0x02;
        break;
    case GYRO_RANGE_1000DPS:
        databyte = 0x01;
        break;
    case GYRO_RANGE_2000DPS:
        databyte = 0x00;
        break;
    }
    gyro_range_ = range;
    if (!i2c_bus_.WriteRegister(FXAS21002C_CTRL_REG0, databyte))
        return false;

    // switch to active mode with 400Hz output  
    databyte = 0x06;
    if (!i2c_bus_.WriteRegister(FXAS21002C_CTRL_REG1, databyte))
        return false;

    return true;
}

double FXAS21002C::GetGyroScale(void)
{
    double unit = 1.0;
    switch (gyro_range_)
    {
    case GYRO_RANGE_250DPS:
        return unit / 131.072;
    case GYRO_RANGE_500DPS:
        return unit / 65.536;
    case GYRO_RANGE_1000DPS:
        return unit / 32.768;
    case GYRO_RANGE_2000DPS:
        return unit / 16.384;
    }

    return 0.0;	
}

bool FXAS21002C::ReadGyroData(FXAS21002_RawData *gyro_data)
{
    // select current device first, in case there are other devices on the bus
    if (!i2c_bus_.SetSlaveAddress(i2c_slave_address_))
        return false;

    uint8_t buffer[FXAS21002C_READ_LEN]; // read buffer

    if (i2c_bus_.ReadBlockData(FXAS21002C_STATUS, FXAS21002C_READ_LEN, buffer) == FXAS21002C_READ_LEN)
    {
        // copy the gyro byte data into 16 bit words
        gyro_data->x = (buffer[1] << 8) | buffer[2];
        gyro_data->y = (buffer[3] << 8) | buffer[4];
        gyro_data->z = (buffer[5] << 8) | buffer[6];
    }
    else
    {
        return false;
    }

    // normal return
    return true;
}