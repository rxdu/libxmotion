/* 
 * vehicle_state.cpp
 * 
 * Created on: Dec 05, 2018 22:32
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "cav_motion/vehicle_state.hpp"

namespace librav
{

std::atomic<int32_t> VehicleState::count = {0};

VehicleState::VehicleState(int32_t id, Pose2d _pose, double _speed, CovarMatrix2d _pos_var, double _spd_var) : id_(id),
                                                                                                               pose_(_pose),
                                                                                                               pos_var_(_pos_var),
                                                                                                               speed_(_speed),
                                                                                                               spd_var_(_spd_var)
{
    footprint_.TransformRT(pose_.position.x, pose_.position.y, pose_.theta);
}

VehicleState::VehicleState(Pose2d _pose, double _speed, CovarMatrix2d _pos_var, double _spd_var) : pose_(_pose),
                                                                                                   pos_var_(_pos_var),
                                                                                                   speed_(_speed),
                                                                                                   spd_var_(_spd_var)
{
    id_ = VehicleState::count;
    VehicleState::count.fetch_add(1);
    footprint_.TransformRT(pose_.position.x, pose_.position.y, pose_.theta);
}

void VehicleState::SetPose(Pose2d ps)
{
    pose_ = ps;
    footprint_.TransformRT(pose_.position.x, pose_.position.y, pose_.theta);
}

} // namespace librav