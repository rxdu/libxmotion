/* 
 * lcm_messenger.hpp
 * 
 * Created on: Oct 31, 2017 11:43
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#ifndef LCM_MESSENGER_HPP
#define LCM_MESSENGER_HPP

#include <memory>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

#include "common/librav_types.hpp"
#include "system/can_messenger.hpp"

namespace librav
{

class LCMMessenger
{
public:
    LCMMessenger(std::shared_ptr<lcm::LCM> lcm);
    ~LCMMessenger() = default;

    void republishRawIMUData(const IMU9DOFData &msg);
    // void republishRawMagData(const pixcar::CarRawMag &msg);
    void republishRawSpeedData(const CarSpeed &msg);

    // void publishCalibratedIMUData(const AccGyroData &imu);
    //void publishCalibratedMagData(const pixcar::CarRawMag &msg);
    void publishConvertedSpeedData(const CarSpeed &spd);

private:
    std::shared_ptr<lcm::LCM> lcm_;
};

}

#endif /* LCM_MESSENGER_HPP */
