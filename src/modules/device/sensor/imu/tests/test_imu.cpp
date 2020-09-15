#include <iostream>
#include "imu/fxos8700cq.hpp"
#include "imu/fxas21002c.hpp"
#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    FXOS8700CQ fxos8700(3);

    if (fxos8700.ConfigI2CBus())
        std::cout << "FXOS8700CQ Sensor connected correctly" << std::endl;
    else
        return -1;
    if (fxos8700.ConfigSensor())
        std::cout << "FXOS8700CQ Sensor initialized" << std::endl;
    else
        return -1;

    FXAS21002C fxas21002(3);

    if (fxas21002.ConfigI2CBus())
        std::cout << "FXAS21002C Sensor connected correctly" << std::endl;
    else
        return -1;
    if (fxas21002.ConfigSensor())
        std::cout << "FXAS21002C Sensor initialized" << std::endl;
    else
        return -1;

    stopwatch::StopWatch stopwatch;

    FXOS8700_RawData accel, mag;
    FXAS21002_RawData gyro;
    while (true)
    {
        stopwatch.tic();

        if (fxos8700.ReadAccelMagnData(&accel, &mag))
            std::cout << "accel: " << accel.x << " , " << accel.y << " , " << accel.z << " ; "
                      << "mag: " << mag.x << " , " << mag.y << " , " << mag.z << std::endl;
        if (fxas21002.ReadGyroData(&gyro))
            std::cout << "gyro: " << gyro.x << " , " << gyro.y << " , " << gyro.z << std::endl;

        std::cout << "---" << std::endl;

        stopwatch.sleep_until_ms(5);
    }

    return 0;
}
