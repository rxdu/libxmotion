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
class TamiyaTA07ProParams
{
    TamiyaTA07ProParams()
    {
        // accel_calib.misalignment_matrix << 1 , 0.000138548,  -0.0096009,
        //                                    0 ,           1,  -1.55922e-05,
        //                                    0 ,           0,            1;

        // accel_calib.scale_matrix << 0.998157,       0,       0,
        //                                  0,     0.998582,    0,
        //                                  0,         0,     0.994014;

        // accel_calib.bias_vector <<  -0.157991, -0.0240133, 0.262363;

        // gyro_calib.misalignment_matrix <<  1,   -0.0157609,     0.00424826,
        //                             -0.0203271,      1,         -0.0746835,
        //                             0.00383096,  0.0432695,          1;

        // gyro_calib.scale_matrix <<  0.0172695,         0,         0,
        //                                 0,          0.0169161,    0,
        //                                 0,             0,    0.0169598;

        // gyro_calib.bias_vector << -0.96199, -0.445437, -0.0504541;
        accel_calib.misalignment_matrix << 1, -0.00989519, -0.00463672,
            0, 1, 0.00150853,
            0, 0, 1;

        accel_calib.scale_matrix << 0.998144, 0, 0,
            0, 0.998575, 0,
            0, 0, 0.994015;

        accel_calib.bias_vector << -0.156653, -0.00601086, 0.332472;

        gyro_calib.misalignment_matrix << 1, -0.00306838, -0.276467,
            -0.131086, 1, 0.0669897,
            0.0496474, -0.00827792, 1;

        gyro_calib.scale_matrix << 0.0129898, 0, 0,
            0, 0.0198754, 0,
            0, 0, 0.0156519;

        gyro_calib.bias_vector << -0.984411, -0.717505, 0.439744;
    };

  public:
    static TamiyaTA07ProParams &GetParams()
    {
        static TamiyaTA07ProParams params;
        return params;
    };

    /* Car Parameters */
    static constexpr double max_steer_angle = 30.0; // in degree

    // wheel diameter: 1.9" (48mm) diameter
    // (0.048 * pi)/(2pi) = 0.024 m/rad
    // 10 m/s ~= 22 mph => 10 / 0.024 ~= 416 rad/s
    static constexpr double max_forward_speed = 420.0; // in rad/s
    static constexpr double max_reverse_speed = -50.0; // in rad/s

    IMUCalibParams accel_calib;
    IMUCalibParams gyro_calib;
};
} // namespace librav

#endif /* IMU_PARAMS_H */
