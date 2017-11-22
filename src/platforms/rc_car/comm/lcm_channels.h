/* 
 * lcm_channels.h
 * 
 * Created on: Oct 31, 2017 11:46
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#ifndef LCM_CHANNELS_H
#define LCM_CHANNELS_H

#include <string>

namespace librav
{

struct LCM_CHANNELS
{
    static const std::string CAR_RAW_IMU_CHANNEL;
    static const std::string CAR_RAW_MAG_CHANNEL;
    static const std::string CAR_RAW_SPEED_CHANNEL;
    
    static const std::string CAR_CALIB_IMU_CHANNEL;
    static const std::string CAR_CALIB_MAG_CHANNEL;
    static const std::string CAR_COVT_SPEED_CHANNEL;
   
    static const std::string CAR_COMMOND_CHANNEL;
};

}

#endif /* LCM_CHANNELS_H */
