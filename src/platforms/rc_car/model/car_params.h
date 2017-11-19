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

#include "common/librav_types.h"

namespace librav
{

class TamiyaTA07ProParams
{
    TamiyaTA07ProParams()
    {
        accel_calib.misalignment_matrix << 1 , 0.000138548,  -0.0096009,
                                           0 ,           1,  -1.55922e-05,
                                           0 ,           0,            1;

        accel_calib.scale_matrix << 0.998157,       0,       0,
                                         0,     0.998582,    0,
                                         0,         0,     0.994014;

        accel_calib.bias_vector <<  -0.157991, -0.0240133, 0.262363;

        gyro_calib.misalignment_matrix <<  1,   -0.0157609,     0.00424826,
                                    -0.0203271,      1,         -0.0746835,
                                    0.00383096,  0.0432695,          1;

        gyro_calib.scale_matrix <<  0.0172695,         0,         0,
                                        0,          0.0169161,    0,
                                        0,             0,    0.0169598;       

        gyro_calib.bias_vector << -0.96199, -0.445437, -0.0504541;
    };

public:
    static TamiyaTA07ProParams& GetParams()
    {
        static TamiyaTA07ProParams params;
        return params;
    };

    IMUCalibParams accel_calib;
    IMUCalibParams gyro_calib;
};

}

#endif /* IMU_PARAMS_H */
