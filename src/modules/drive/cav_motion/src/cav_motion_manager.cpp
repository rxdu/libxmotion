/* 
 * cav_motion_manager.cpp
 * 
 * Created on: Dec 06, 2018 03:21
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "cav_motion/cav_motion_manager.hpp"

#include <iostream>

#include "stopwatch/stopwatch.h"

using namespace autodrive;

CAVMotionManager::CAVMotionManager(std::string map_file) : map_loader_(map_file)
{
    // setup communication link
    data_link_ = std::make_shared<LCMLink>();
    if (!data_link_->good())
        std::cerr << "ERROR: Failed to initialize LCM." << std::endl;
    data_link_ready_ = true;

    data_link_->subscribe(CAV_COMMON_CHANNELS::VEHICLE_ESTIMATIONS_CHANNEL, &CAVMotionManager::HandleVehicleEstimationsMsg, this);
    data_link_->subscribe(CAV_COMMON_CHANNELS::EGO_VEHICLE_STATE_CHANNEL, &CAVMotionManager::HandleEgoVehicleStateMsg, this);
    data_link_->subscribe(CAV_COMMON_CHANNELS::CAV_MISSION_INFO_CHANNEL, &CAVMotionManager::HandleMissionInfoMsg, this);
}

bool CAVMotionManager::IsReady()
{
    if (!data_link_ready_)
        return false;

    if (!map_loader_.map_ready)
        return false;

    route_planner_ = std::make_shared<RoutePlanner>(map_loader_.road_map, map_loader_.traffic_map);

    return true;
}

void CAVMotionManager::ResetTask()
{
    current_route_.valid = false;
}

void CAVMotionManager::HandleMissionInfoMsg(const autodrive::ReceiveBuffer *rbuf, const std::string &chan, const librav_lcm_msgs::CAVMissionInfo *msg)
{
    if (msg->mission_type == librav_lcm_msgs::CAVMissionInfo::MT_NEW_MISSION_REQUEST)
    {
        position_s_.x = msg->start_state.position[0];
        position_s_.y = msg->start_state.position[1];
        position_g_.x = msg->goal_state.position[0];
        position_g_.y = msg->goal_state.position[1];

        new_mission_request_received_ = true;

        std::cout << "new mission request received" << std::endl;
    }
    else if (msg->mission_type == librav_lcm_msgs::CAVMissionInfo::MT_ABORT_CURRENT_MISSION)
    {
        abort_mission_request_received_ = true;

        std::cout << "abort mission request received" << std::endl;
    }
}

void CAVMotionManager::HandleEgoVehicleStateMsg(const autodrive::ReceiveBuffer *rbuf, const std::string &chan, const librav_lcm_msgs::VehicleState *msg)
{
    ego_vehicle_state_ = VehicleState(msg->id, {msg->position[0], msg->position[1], msg->theta}, msg->speed);
}

void CAVMotionManager::HandleVehicleEstimationsMsg(const autodrive::ReceiveBuffer *rbuf, const std::string &chan, const librav_lcm_msgs::VehicleEstimations *msg)
{
    surrounding_vehicles_.clear();
    for (int i = 0; i < msg->vehicle_num; ++i)
    {
        auto est = msg->estimations[i];
        surrounding_vehicles_.emplace_back(VehicleState(est.id, {est.position[0], est.position[1], est.theta}, est.speed));
    }

    new_veh_est_received_ = true;
}

void CAVMotionManager::Run()
{
    std::cout << "INFO: motion manager started..." << std::endl;

    stopwatch::StopWatch planner_stopwatch;

    while (true)
    {
        // generate new route if requested
        if (new_mission_request_received_)
        {
            std::cout << "look for route" << std::endl;
            bool res = route_planner_->SearchRoute({position_s_.x, position_s_.y}, {position_g_.x, position_g_.y}, &current_route_);

            if (res)
            {
                current_route_.valid = true;
                std::cout << "route generated" << std::endl;
            }
            new_mission_request_received_ = false;
        }

        if (abort_mission_request_received_)
        {
            abort_mission_request_received_ = false;
        }

        // by default look for motion plan according to desired route
        if (new_veh_est_received_ && current_route_.valid)
        {
            planner_stopwatch.tic();

            new_veh_est_received_ = false;
            std::cout << "planning iteration finished in " << planner_stopwatch.mtoc() << " seconds" << std::endl;
        }

        data_link_->handleTimeout(0);
    }
}