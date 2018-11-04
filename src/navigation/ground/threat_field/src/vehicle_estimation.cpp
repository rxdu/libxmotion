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

int32_t VehicleEstimation::VehicleCount = 0;

VehicleEstimation::VehicleEstimation()
{
    id_ = ++VehicleEstimation::VehicleCount;
}

VehicleEstimation::VehicleEstimation(Pose2d _pose, double _speed) : pose(_pose), speed(_speed)
{
    id_ = ++VehicleEstimation::VehicleCount;
    footprint.TransformRT(pose.position.x, pose.position.y, pose.theta);
}

VehicleEstimation::VehicleEstimation(Pose2d _pose, double _speed, std::shared_ptr<TrafficChannel> chn) : pose(_pose), speed(_speed), occupied_channels_({chn})
{
    id_ = ++VehicleEstimation::VehicleCount;
    footprint.TransformRT(pose.position.x, pose.position.y, pose.theta);
}

VehicleEstimation::VehicleEstimation(Pose2d _pose, double _speed, std::vector<std::shared_ptr<TrafficChannel>> chns) : pose(_pose), speed(_speed), occupied_channels_(chns)
{
    id_ = ++VehicleEstimation::VehicleCount;
    footprint.TransformRT(pose.position.x, pose.position.y, pose.theta);
}

void VehicleEstimation::SetPose(Pose2d ps)
{
    pose = ps;
    footprint.TransformRT(pose.position.x, pose.position.y, pose.theta);
}