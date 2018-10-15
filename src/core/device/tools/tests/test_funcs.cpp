#include <iostream>

extern "C"
{
    #include "i2c-tools/i2cbusses.h"
}

int main()
{
    std::cout << "Test funcs: " << std::endl;

    char bus_name[3] = "3";
    int i2cbus = lookup_i2c_bus(bus_name);
    std::cout << "lookup result: " << i2cbus << std::endl;

    return 0;
}