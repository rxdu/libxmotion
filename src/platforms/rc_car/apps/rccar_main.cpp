/* 
 * car_control.cpp
 * 
 * Created on: Oct 30, 2017 22:40
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 * 
 * Description: this node manages the communication between the PC/SBC and the MCU on car.
 *      MCU -> PC/SBC: a callback function will be called, receive new sensor data, update control
 *      PC/SBC -> MCU: send control commands of servo and motor
 * 
 *      (real-time performance of this node is desired)
 *  
 */

#include <functional>

// #define ENABLE_CSV_LOGGING

#include "system/nerve_system.hpp"

#define WHEEL_DIAMETER 0.065
#define GEAR_RATIO 6.58 // with 20T pinion gear

using namespace librav;

int main(int argc, char *argv[])
{
    std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

    if (!lcm->good())
    {
        std::cout << "ERROR: Failed to initialize LCM." << std::endl;
        return -1;
    }

    NerveSystem nervesys(lcm);

    if (!nervesys.init())
        return -1;

    std::cout << "Start RC Car System Main Control Loop" << std::endl;
    nervesys.start();

    return 0;
}