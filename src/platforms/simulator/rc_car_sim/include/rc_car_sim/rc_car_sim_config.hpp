/* 
 * rc_car_sim_config.hpp
 * 
 * Created on: Aug 9, 2016
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef RC_CAR_SIM_CONFIG_HPP
#define RC_CAR_SIM_CONFIG_HPP

namespace librav
{

// RC car model C
#ifdef RC_CAR_MODEL_C

#define IMG_RES_X 160
#define IMG_RES_Y 90

#define LASER_SCAN_RES_X 64
#define LASER_SCAN_RES_y 64

// Default model parameters
#else

#define IMG_RES_X 160
#define IMG_RES_Y 90

#define LASER_SCAN_RES_X 64
#define LASER_SCAN_RES_y 64

#endif

const double max_steer_angle = 30.0;  // in degree
const double max_drive_speed = 100.0; // in round/s
}

#endif /* RC_CAR_SIM_CONFIG_HPP */
