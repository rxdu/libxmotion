#include <iostream>
#include "imu/fxos8700cq.hpp"
#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    FXOS8700CQ fxos8700(3);

    if(fxos8700.ConfigI2CBus())
        std::cout << "Sensor connected correctly" << std::endl;
    else
        return -1;
    if(fxos8700.ConfigSensor())
        std::cout << "Sensor initialized" << std::endl;
    else
        return -1;
        
    stopwatch::StopWatch stopwatch;
    
    FXOS8700_RawData accel, mag;
    while(true)
    {
        stopwatch.tic();
        fxos8700.ReadAccelMagnData(&accel, &mag);

        std::cout << "accel: " << accel.x << " , " << accel.y << " , " << accel.z << " ; "
                << "mag: " << mag.x << " , " << mag.y << " , " << mag.z << std::endl;
    
        stopwatch.sleep_until_ms(5);
    }
   
    return 0;
}
