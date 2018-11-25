/* 
 * car_params.h
 * 
 * Created on: Nov 13, 2017 15:02
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef IMU_PARAMS_H
#define IMU_PARAMS_H

#include "common/librav_types.hpp"

namespace librav
{
#define RS_RGB_IMG_RES_X 1920
#define RS_RGB_IMG_RES_Y 1080

#define RS_DEP_IMG_RES_X 1280
#define RS_DEP_IMG_RES_Y 720

#define LASER_SCAN_RES_X 64
#define LASER_SCAN_RES_y 64

struct TamiyaTA07ProSimParams
{
    // static TamiyaTA07ProSimParams &GetParams()
    // {
    //     static TamiyaTA07ProSimParams params;
    //     return params;
    // };

    /* Car Parameters */
    static constexpr double max_steer_angle = 30.0; // in degree

    // wheel diameter: ~6.67cm diameter
    // (0.067 * pi)/(2pi) = 0.0335 m/rad
    // 10 m/s ~= 22 mph => 10 / 0.0335 ~= 300 rad/s
    // 5 m/s ~= 11 mph => 5 / 0.0335 ~= 150 rad/s => for more stable control
    static constexpr double max_forward_speed = 150.0; // in rad/s
    static constexpr double max_reverse_speed = -50.0; // in rad/s

    /* Sensor Parameters */
    static constexpr int32_t main_cam_res_x = RS_RGB_IMG_RES_X;
    static constexpr int32_t main_cam_res_y = RS_RGB_IMG_RES_Y;
};
} // namespace librav

#endif /* IMU_PARAMS_H */
