/* 
 * cav_motion_manager.cpp
 * 
 * Created on: Dec 06, 2018 03:21
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "cav_motion/cav_motion_manager.hpp"

#include <iostream>

using namespace librav;

CAVMotionManager::CAVMotionManager()
{
    // setup communication link
    data_link_ = std::make_shared<LCMLink>();
    if (!data_link_->IsGood())
        std::cerr << "ERROR: Failed to initialize LCM." << std::endl;
    data_link_ready_ = true;
}

bool CAVMotionManager::IsReady()
{
    if (!data_link_ready_)
        return false;

    return true;
}