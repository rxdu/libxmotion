/* 
 * i2c.cpp
 * 
 * Created on: Jan 08, 2018 22:51
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "i2c-bus/i2c.hpp"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/param.h> /* for NAME_MAX */
#include <unistd.h>
#include <limits.h>
#include <dirent.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include <iostream>

using namespace librav;

I2CBus::I2CBus(int bus_id) : bus_id_(bus_id)
{
}

I2CBus::~I2CBus()
{
    if (bus_initialized_)
        close(bus_fd_);
}

bool I2CBus::Init(int bus_id)
{
    if (bus_id_ == -1 && bus_id == -1)
    {
        std::cerr << "Invalid I2C bus ID: " << static_cast<int32_t>(bus_id_) << std::endl;
        return false;
    }
    else
    {
        bus_id_ = bus_id;
    }
    bus_name_ = "/dev/i2c-" + std::to_string(bus_id_);

    bus_fd_ = open(bus_name_.c_str(), O_RDWR);
    if (bus_fd_ < 0)
    {
        std::cerr << "Failed to open: " << bus_name_ << std::endl;
        return false;
    }
    else
    {
        std::cout << "Device " << bus_name_ << " opened successfully" << std::endl;
    }

    bus_initialized_ = true;

    return true;
}

bool I2CBus::SetSlaveAddress(int slave_addr, bool force)
{
    if (ioctl(bus_fd_, force ? I2C_SLAVE_FORCE : I2C_SLAVE, slave_addr) < 0)
    {
        std::cerr << "Failed to set slave address " << slave_addr << " for " << bus_name_ << std::endl;
        return false;
    }

    slave_address_ = slave_addr;

    return true;
}

bool I2CBus::ReadRegister(int reg_addr, uint8_t *data)
{
    int res = i2c_smbus_read_byte_data(bus_fd_, reg_addr);
    if (res < 0)
    {
        std::cout << "Error - byte read failed from register " << reg_addr << std::endl;
        return false;
    }

    *data = res;
    return true;
}

int32_t I2CBus::ReadBlockData(int reg_addr_base, int data_len, uint8_t *data)
{
    int32_t res = i2c_smbus_read_i2c_block_data(bus_fd_, reg_addr_base, data_len, data);
    if (res < 0)
        std::cout << "Error - block read failed from register base " << reg_addr_base << std::endl;

    return res;
}

bool I2CBus::WriteRegister(int reg_addr, int data)
{
    int res = i2c_smbus_write_byte_data(bus_fd_, reg_addr, data);
    if (res < 0)
    {
        std::cout << "Error - byte write failed from register " << reg_addr << std::endl;
        return false;
    }

    return true;
}