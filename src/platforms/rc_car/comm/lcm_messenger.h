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

#include "common/librav_types.hpp"
#include "comm/can_messenger.h"

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

    void publishCalibratedIMUData(const AccGyroData &imu);
    //void publishCalibratedMagData(const pixcar::CarRawMag &msg);
    void publishConvertedSpeedData(const CarSpeed &spd);

private:
    std::shared_ptr<lcm::LCM> lcm_;
};

}

#endif /* LCM_MESSENGER_H */
