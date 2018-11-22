#include <iostream>
#include "sensors/fxas21002c.hpp"
#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    FXAS21002C fxas21002(3);

    if (fxas21002.ConfigI2CBus())
        std::cout << "Sensor connected correctly" << std::endl;
    else
        return -1;
    if (fxas21002.ConfigSensor())
        std::cout << "Sensor initialized" << std::endl;
    else
        return -1;

    stopwatch::StopWatch stopwatch;

    FXAS21002_RawData gyro;
    while (true)
    {
        stopwatch.tic();
        if (fxas21002.ReadGyroData(&gyro))
            std::cout << "gyro: " << gyro.x << " , " << gyro.y << " , " << gyro.z << std::endl;

        stopwatch.sleep_until_ms(5);
    }

    return 0;
}
