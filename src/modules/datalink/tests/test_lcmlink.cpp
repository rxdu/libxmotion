#include <memory>
#include <iostream>

#include "datalink/lcm_link.hpp"

using namespace ivnav;

int main(int argc, char *argv[])
{
    std::shared_ptr<LCMLink> link = std::make_shared<LCMLink>();

    if (!link->good())
    {
        std::cout << "ERROR: Failed to initialize LCM." << std::endl;
        return -1;
    }

    librav_lcm_msgs::CarRawSpeed_t spd_msg;

    spd_msg.mtime = 1;
    spd_msg.count = 10;
    spd_msg.speed = 15;

    link->publish("test_channel", &spd_msg);

    return 0;
}