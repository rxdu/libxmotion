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
        data = 1;
        i2cbus.WriteRegister(0x2a, data);

        if (i2cbus.ReadRegister(0x2a, &data))
            std::cout << "expect 1, read result: " << std::hex << data << std::endl;
        else
            std::cout << "Failed to read" << std::endl;

        data = 0;
        i2cbus.WriteRegister(0x2a, data);

        if (i2cbus.ReadRegister(0x2a, &data))
            std::cout << "expect 0, read result: " << data << std::endl;
        else
            std::cout << "Failed to read" << std::endl;

        data = 1;
        i2cbus.WriteRegister(0x2a, data);

        if (i2cbus.ReadRegister(0x2a, &data))
            std::cout << "expect 1, read result: " << data << std::endl;
        else
            std::cout << "Failed to read" << std::endl;
    }

    return 0;
}