/*
 * test_interface.cpp
 *
 *  Created on: Dec 25, 2016
 *      Author: rdu
 */

#include <iostream>

#include "async_serial/async_serial.hpp"
#include "gps/gps_receiver.hpp"

#include "datalink/lcm_link.hpp"

using namespace librav;

int main(int argc, char *argv[])
{
    std::shared_ptr<LCMLink> link = std::make_shared<LCMLink>();

    if (!link->IsGood())
    {
        std::cout << "ERROR: Failed to initialize LCM." << std::endl;
        return -1;
    }

    std::shared_ptr<GPSReceiver> gps = std::make_shared<GPSReceiver>("/dev/ttyUSB0", 115200, link);

    if (gps->Connected())
        std::cout << "GPS connected" << std::endl;

    while (1)
    {
        link->handleTimeout(0);
    }
}
