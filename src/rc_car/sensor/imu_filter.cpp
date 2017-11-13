/* 
 * imu_filter.cpp
 * 
 * Created on: Nov 13, 2017 15:28
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#include "rc_car/sensor/imu_filter.h"
#include "rc_car/model/car_params.h"

using namespace librav;

IMUFilter::IMUFilter()
{
    // use parameters for Tamiya TA07 Pro model
    accel_calib_ = TamiyaTA07ProParams::GetParams().accel_calib;
    gyro_calib_ = TamiyaTA07ProParams::GetParams().accel_calib;
}

AccGyroData IMUFilter::CorrectIMURawData(const AccGyroData& raw)
{
    Eigen::Matrix<double,3,1> raw_acc, raw_gyro;
    raw_acc << raw.accel.x , raw.accel.y , raw.accel.z;
    raw_gyro << raw.gyro.x, raw.gyro.y, raw.gyro.z;

    Eigen::Matrix<double,3,1> cor_acc, cor_gyro;

    cor_acc = accel_calib_.misalignment_matrix * accel_calib_.scale_matrix * (raw_acc + accel_calib_.bias_vector);
    cor_gyro = gyro_calib_.misalignment_matrix * gyro_calib_.scale_matrix * (raw_gyro + gyro_calib_.bias_vector);

    return AccGyroData(raw.mtime, cor_acc(0), cor_acc(1), cor_acc(2), cor_gyro(0), cor_gyro(1), cor_gyro(2));
}