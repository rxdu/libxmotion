/* 
 * vehicle_estimation.cpp
 * 
 * Created on: Nov 02, 2018 08:36
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "prediction/vehicle_estimation.hpp"

using namespace autodrive;

int32_t VehicleEstimation::VehicleCount = 0;

VehicleEstimation::VehicleEstimation()
{
    id_ = ++VehicleEstimation::VehicleCount;
}

VehicleEstimation::VehicleEstimation(Pose2d _pose, double _speed) : pose_(_pose), speed_(_speed)
{
    id_ = ++VehicleEstimation::VehicleCount;
    footprint_.TransformRT(pose_.position.x, pose_.position.y, pose_.theta);
}

VehicleEstimation::VehicleEstimation(Pose2d _pose, CovarMatrix2d _pos_var, double _speed, double _spd_var) : pose_(_pose),
                                                                                                             pos_var_(_pos_var),
                                                                                                             speed_(_speed),
                                                                                                             spd_var_(_spd_var)
{
    id_ = ++VehicleEstimation::VehicleCount;
    footprint_.TransformRT(pose_.position.x, pose_.position.y, pose_.theta);
}

void VehicleEstimation::SetPose(Pose2d ps)
{
    pose_ = ps;
    footprint_.TransformRT(pose_.position.x, pose_.position.y, pose_.theta);
}