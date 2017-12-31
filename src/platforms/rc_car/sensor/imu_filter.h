/* 
 * imu_filter.h
 * 
 * Created on: Nov 13, 2017 15:00
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#ifndef IMU_FILTER_H
#define IMU_FILTER_H

#include "common/librav_types.hpp"

namespace librav
{

class IMUFilter
{
public:
    IMUFilter();
    ~IMUFilter() = default;

public:
    AccGyroData CorrectIMURawData(const AccGyroData& raw);

private:
    IMUCalibParams accel_calib_;
    IMUCalibParams gyro_calib_;
};

}

#endif /* IMU_FILTER_H */
