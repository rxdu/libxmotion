#include <iostream>
#include <cstdint>
#include <vector>

#include "stopwatch/stopwatch.h"
// #include "common/librav_types.hpp"
#include "system/can_messenger.hpp"

using namespace librav;

int main()
{
    CANMessenger msger("can0");
    stopwatch::StopWatch timer;

	uint32_t loop = 0;
    while(true)
    {
        // timer.tic();
//		if(loop++ % 10)
			msger.SendActuatorCmdToCar(0.15, 0.28, 0);
        msger.Spin(2);
        // timer.sleep_util_ms(1000);
    }
}
