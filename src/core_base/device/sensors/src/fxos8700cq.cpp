/* 
 * fxos8700cq.cpp
 * 
 * Created on: Jan 12, 2018 21:18
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "sensors/fxos8700cq.hpp"

using namespace librav;

FXOS8700CQ::FXOS8700CQ(int32_t i2c_dev_id, int32_t i2c_addr) : i2c_device_id_(i2c_dev_id),
                                                               i2c_slave_address_(i2c_addr)
{
}

bool FXOS8700CQ::ConfigI2CBus()
{
    if (!i2c_bus_.Init(i2c_device_id_))
        return false;

    if (!i2c_bus_.SetSlaveAddress(i2c_slave_address_))
        return false;

    // read and check the FXOS8700CQ WHOAMI register
    uint8_t databyte;
    if (!i2c_bus_.ReadRegister(FXOS8700CQ_WHOAMI, &databyte))
        return false;

    if (databyte != FXOS8700CQ_WHOAMI_VAL)
        return false;

    return true;
}

bool FXOS8700CQ::ConfigSensor(FXOS8700AccelRange range)
{
    uint8_t databyte;

    // write 0000 0000 = 0x00 to accelerometer control register 1 to
    // place FXOS8700CQ into standby
    // [7-1] = 0000 000
    // [0]: active=0
    databyte = 0x00;
    if (!i2c_bus_.WriteRegister(FXOS8700CQ_CTRL_REG1, databyte))
        return false;

    // write 0001 1111 = 0x1F to magnetometer control register 1
    // [7]: m_acal=0: auto calibration disabled
    // [6]: m_rst=0: no one-shot magnetic reset
    // [5]: m_ost=0: no one-shot magnetic measurement
    // [4-2]: m_os=111=7: 8x oversampling (for 200Hz) to reduce
    // magnetometer noise
    // [1-0]: m_hms=11=3: select hybrid mode with accel and
    // magnetometer active
    databyte = 0x1F;
    if (!i2c_bus_.WriteRegister(FXOS8700CQ_M_CTRL_REG1, databyte))
        return false;

    // write 0010 0000 = 0x20 to magnetometer control register 2
    // [7]: reserved
    // [6]: reserved
    // [5]: hyb_autoinc_mode=1 to map the magnetometer registers to
    // follow the accelerometer registers
    // [4]: m_maxmin_dis=0 to retain default min/max latching even
    // though not used
    // [3]: m_maxmin_dis_ths=0
    // [2]: m_maxmin_rst=0
    // [1-0]: m_rst_cnt=00 to enable magnetic reset each cycle
    databyte = 0x20;
    if (!i2c_bus_.WriteRegister(FXOS8700CQ_M_CTRL_REG2, databyte))
        return false;

    // write 0000 0001= 0x01 to XYZ_DATA_CFG register
    // [7]: reserved
    // [6]: reserved
    // [5]: reserved
    // [4]: hpf_out=0
    // [3]: reserved
    // [2]: reserved
    // [1-0]: fs=01 for accelerometer range of +/-4g range with
    // 0.488mg / LSB
    /* Configure the accelerometer */
    switch (range)
    {
    case (ACCEL_RANGE_2G):
        databyte = 0x00;
        break;
    case (ACCEL_RANGE_4G):
        databyte = 0x01;
        break;
    case (ACCEL_RANGE_8G):
        databyte = 0x02;
        break;
    }
    accel_range_ = range;
    if (!i2c_bus_.WriteRegister(FXOS8700CQ_XYZ_DATA_CFG, databyte))
        return false;

    // write 0000 1101 = 0x0D to accelerometer control register 1
    // [7-6]: aslp_rate=00
    // [5-3]: dr=000 for 400Hz data rate (when in hybrid mode)
    // [2]: lnoise=1 for low noise mode
    // [1]: f_read=0 for normal 16 bit reads
    // [0]: active=1 to take the part out of standby and enable
    //sampling
    databyte = 0x05;
    if (!i2c_bus_.WriteRegister(FXOS8700CQ_CTRL_REG1, databyte))
        return false;

    // normal return
    return true;
}

double FXOS8700CQ::GetAccelScale()
{
    switch (accel_range_)
    {
    case (ACCEL_RANGE_2G):
        return gravity_const / 4096.0;
    case (ACCEL_RANGE_4G):
        return gravity_const / 2048.0;
    case (ACCEL_RANGE_8G):
        return gravity_const / 1024.0;
    }

    return 0.0;
}

double FXOS8700CQ::GetMagnScale()
{
    double unit = 1.0; // uT
    return unit/27.307;
}

// read status and the three channels of accelerometer and
// magnetometer data from
// FXOS8700CQ (13 bytes)
bool FXOS8700CQ::ReadAccelMagnData(FXOS8700_RawData *acc_data, FXOS8700_RawData *mag_data)
{
    // select current device first, in case there are other devices on the bus
    if (!i2c_bus_.SetSlaveAddress(i2c_slave_address_))
        return false;

    uint8_t buffer[FXOS8700CQ_READ_LEN]; // read buffer

    if (i2c_bus_.ReadBlockData(FXOS8700CQ_STATUS, FXOS8700CQ_READ_LEN, buffer) == FXOS8700CQ_READ_LEN)
    {
        // copy the 14 bit accelerometer byte data into 16 bit words
        acc_data->x = (int16_t)(((buffer[1] << 8) | buffer[2])) >> 2;
        acc_data->y = (int16_t)(((buffer[3] << 8) | buffer[4])) >> 2;
        acc_data->z = (int16_t)(((buffer[5] << 8) | buffer[6])) >> 2;

        // copy the magnetometer byte data into 16 bit words
        mag_data->x = (buffer[7] << 8) | buffer[8];
        mag_data->y = (buffer[9] << 8) | buffer[10];
        mag_data->z = (buffer[11] << 8) | buffer[12];
    }
    else
    {
        return false;
    }

    // normal return
    return true;
}