/* 
 * vehicle_estimation.cpp
 * 
 * Created on: Nov 02, 2018 08:36
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "threat_field/vehicle_estimation.hpp"

using namespace librav;

VehicleEstimation::VehicleEstimation(Pose2d _pose, double _speed) : pose(_pose), speed(_speed)
{
    footprint.TransformRT(pose.position.x, pose.position.y, pose.theta);
}

void VehicleEstimation::SetPose(Pose2d ps)
{
    pose = ps;
    footprint.TransformRT(pose.position.x, pose.position.y, pose.theta);
}