/* 
 * read_whoami.cpp
 * 
 * Created on: Jan 08, 2018 23:16
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include <iostream>
#include "imu/i2c.hpp"

using namespace librav;

int main()
{
    I2CBus i2cbus(3);

    bool res = i2cbus.Init();
    if (!res)
        return -1;

    uint8_t data;
    if (i2cbus.SetSlaveAddress(0x1e))
    {
        if (i2cbus.ReadRegister(0x0d, &data))
            std::cout << "result: " << data << std::endl;
        else
            std::cout << "Failed to read" << std::endl;
    }

    return 0;
}