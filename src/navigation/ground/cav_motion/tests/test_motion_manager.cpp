#include <iostream>
#include <cstdint>
#include <cmath>

#include "road_map/road_map.hpp"
#include "traffic_map/map_loader.hpp"
#include "cav_motion/cav_motion_manager.hpp"

#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    // std::shared_ptr<LCMLink> data_link_ = std::make_shared<LCMLink>();
    // if (!data_link_->good())
    //     std::cerr << "ERROR: Failed to initialize LCM." << std::endl;

    CAVMotionManager planner;

    if (!planner.IsReady())
    {
        std::cerr << "ERROR: planner is not set up properly" << std::endl;
        return -1;
    }

    return 0;
}