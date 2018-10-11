/* 
 * lcm_channels.cpp
 * 
 * Created on: Oct 31, 2017 11:50
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#include "system/lcm_channels.hpp"

namespace librav
{

    const std::string LCM_CHANNELS::CAR_RAW_IMU_CHANNEL =  "car_raw_imu";
    const std::string LCM_CHANNELS::CAR_RAW_MAG_CHANNEL =  "car_raw_mag";
    const std::string LCM_CHANNELS::CAR_RAW_SPEED_CHANNEL = "car_raw_speed";

    const std::string LCM_CHANNELS::CAR_CALIB_IMU_CHANNEL =  "car_calib_imu";
    const std::string LCM_CHANNELS::CAR_CALIB_MAG_CHANNEL =  "car_calib_mag";
    const std::string LCM_CHANNELS::CAR_COVT_SPEED_CHANNEL = "car_covt_speed";

    const std::string LCM_CHANNELS::CAR_COMMOND_CHANNEL = "car_command";
}