/* 
 * lcm_messenger.h
 * 
 * Created on: Oct 31, 2017 11:43
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#ifndef LCM_MESSENGER_H
#define LCM_MESSENGER_H

#include <memory>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

#include "rc_car/comm/can_messenger.h"

namespace librav
{

class LCMMessenger
{
public:
    LCMMessenger(std::shared_ptr<lcm::LCM> lcm);
    ~LCMMessenger() = default;

    void republishRawIMUData(const pixcar::CarRawIMU &msg);
    void republishRawMagData(const pixcar::CarRawMag &msg);
    void republishRawSpeedData(const pixcar::CarRawSpeed &msg);

private:
    std::shared_ptr<lcm::LCM> lcm_;
};

}

#endif /* LCM_MESSENGER_H */
