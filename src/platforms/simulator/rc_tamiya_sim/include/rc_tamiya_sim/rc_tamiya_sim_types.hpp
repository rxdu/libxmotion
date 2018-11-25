/* 
 * rc_tamiya_sim_types.hpp
 * 
 * Created on: Aug 10, 2017
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef RC_TAMIYA_SIM_TYPES_HPP
#define RC_TAMIYA_SIM_TYPES_HPP

#include "common/librav_types.hpp"
#include "rc_tamiya_sim/rc_tamiya_sim_params.hpp"

namespace librav
{
struct DataFromRCTamiyaSim
{
    // sensor data
    unsigned char mono_image[RS_RGB_IMG_RES_Y][RS_RGB_IMG_RES_X];
    IMU6DOFData imu_data;

    float body_speed;
    Pose2f body_pose;

    float speed_rear_right;
    float speed_rear_left;
    float speed_front_right;
    float speed_front_left;

    float steering_angle;
};

struct DataToRCTamiyaSim
{
    float driving_cmd;
    float steering_cmd;
};
} // namespace librav

#endif /* RC_TAMIYA_SIM_TYPES_HPP */
