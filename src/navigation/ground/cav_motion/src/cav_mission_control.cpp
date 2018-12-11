/* 
 * cav_mission_control.cpp
 * 
 * Created on: Dec 09, 2018 22:52
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "cav_motion/cav_mission_control.hpp"

#include "cav_common/vehicle_state.hpp"

namespace librav
{
CAVMissionControl::CAVMissionControl(std::string map_file) : map_loader_(map_file)
{
    // setup map
    map_ready_ = map_loader_.map_ready;
    if (!map_ready_)
    {
        std::cerr << "ERROR: Failed to initialize map." << std::endl;
    }
    else
    {
        road_map_ = map_loader_.road_map;
        traffic_map_ = map_loader_.traffic_map;
        route_planner_ = std::make_shared<RoutePlanner>(map_loader_.road_map, map_loader_.traffic_map);
    }

    // setup communication link
    data_link_ = std::make_shared<LCMLink>();
    if (!data_link_->good())
        std::cerr << "ERROR: Failed to initialize LCM." << std::endl;
    else
        data_link_ready_ = true;
}

bool CAVMissionControl::IsReady()
{
    return (map_ready_ && data_link_ready_);
}

void CAVMissionControl::GenerateMissionInitialState(std::string src, std::string dst, double s, double delta)
{
    position_s_ = traffic_map_->FindTrafficChannel(src, dst)->ConvertToGlobalCoordinate({s, delta});
    pose_s_ = traffic_map_->FindTrafficChannel(src, dst)->ConvertToCurvePoint({s, delta});

    init_state_set_ = true;
}

void CAVMissionControl::GenerateMissionGoalState(std::string src, std::string dst, double s, double delta)
{
    position_g_ = traffic_map_->FindTrafficChannel(src, dst)->ConvertToGlobalCoordinate({s, delta});
    pose_g_ = traffic_map_->FindTrafficChannel(src, dst)->ConvertToCurvePoint({s, delta});

    goal_state_set_ = true;

    if (init_state_set_ && goal_state_set_)
    {
        ValidateMissionRequest();
    }
}

void CAVMissionControl::Run()
{
    std::cout << "INFO: mission control started..." << std::endl;

    while (true)
    {
        // Note:
        //      1. Normally "mission control" should wait for new missions to come
        //      if no valide mission is available and otherwise send the mission to
        //      motion planners and then wait for next requests(new mission/cancel
        //      mission etc.).
        //      2. Here the mission control and motion manager are coupled into one
        if (mission_validated_)
        {
            VehicleState state(-1, {pose_s_.x, pose_s_.y, pose_s_.theta}, 0);
            auto pose = state.GetPose();
            auto fp = state.GetFootprint();

            // send mission information to planner
            {
                librav_lcm_msgs::CAVMissionInfo mission_msg;

                mission_msg.start_state.id = state.id_;
                mission_msg.start_state.position[0] = position_s_.x;
                mission_msg.start_state.position[1] = position_s_.y;

                mission_msg.goal_state.position[0] = position_g_.x;
                mission_msg.goal_state.position[1] = position_g_.y;

                mission_msg.mission_type = librav_lcm_msgs::CAVMissionInfo::MT_NEW_MISSION_REQUEST;

                data_link_->publish(CAV_COMMON_CHANNELS::CAV_MISSION_INFO_CHANNEL, &mission_msg);

                std::cout << "new mission request sent" << std::endl;
            }

            // normally this state msg should be sent by perception system
            {
                librav_lcm_msgs::VehicleState state_msg;

                state_msg.id = state.id_;
                state_msg.position[0] = pose.position.x;
                state_msg.position[1] = pose.position.y;
                state_msg.theta = pose.theta;
                state_msg.speed = state.GetSpeed();

                for (int i = 0; i < 4; ++i)
                {
                    auto pt = fp.GetPoint(i);
                    state_msg.footprint.points[i].x = pt.x;
                    state_msg.footprint.points[i].y = pt.y;
                }
                data_link_->publish(CAV_COMMON_CHANNELS::EGO_VEHICLE_STATE_CHANNEL, &state_msg);
            }

            // std::cout << "init position: " << position_s_.x << position_s_.y << " ; "
            //           << pose.position.x << " , " << pose.position.y << std::endl;

            // TODO:
            // [delegate mission to motion planners]
            // [cancel mission if requested]
            //  => send a LCM message to motion planners
        }

        // simply return in this simplified setup
        return;

        // TODO: wait for new mission requests

        data_link_->handleTimeout(0);
    }
}

void CAVMissionControl::ValidateMissionRequest()
{
    bool res = route_planner_->SearchRoute({position_s_.x, position_s_.y}, {position_g_.x, position_g_.y}, &latest_requested_route_);

    if (res)
        mission_validated_ = true;
    else
        mission_validated_ = false;
}

void CAVMissionControl::ResetMission()
{
    init_state_set_ = false;
    goal_state_set_ = false;
}
} // namespace librav