#include <iostream>
#include <cstdint>
#include <vector>

#include "driver/imu_neo.hpp"
#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    IMUNeo imu;
    stopwatch::StopWatch timer;

    IMU9DOFData imu_data;
    if (imu.InitIMU())
    {
        while (true)
        {
            timer.tic();
            imu.GetIMUData(&imu_data);
            timer.sleep_until_ms(20);
        }
    }
    else
    {
        return -1;
    }

    return 0;
}