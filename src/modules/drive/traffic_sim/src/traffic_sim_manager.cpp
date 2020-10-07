/* 
 * traffic_sim_manager.cpp
 * 
 * Created on: Dec 05, 2018 21:29
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "traffic_sim/traffic_sim_manager.hpp"

#include <cmath>
#include <iostream>

#include "cav_motion/cav_datalink.hpp"

using namespace librav;

TrafficSimManager::TrafficSimManager(TrafficSimConfig config) : config_(config), map_loader_(config.map)
{
    // setup communication link
    data_link_ = std::make_shared<LCMLink>();
    if (!data_link_->good())
        std::cerr << "ERROR: Failed to initialize LCM." << std::endl;
    data_link_ready_ = true;

    data_link_->subscribe(CAV_SIM_CHANNELS::SIM_SYNC_TRIGGER_CHANNEL, &TrafficSimManager::HandleSyncTriggerMsg, this);
}

bool TrafficSimManager::ValidateSimConfig()
{
    // check datalink status
    if (!data_link_ready_)
        return false;

    // check if map loaded successfully
    if (!map_loader_.map_ready)
        return false;

    // add vehicle init states
    vehicle_manager_ = std::make_shared<VehicleManager>(map_loader_.traffic_map);
    vehicle_manager_->AddVehicles(config_.surrounding_vehicles);

    return true;
}

void TrafficSimManager::HandleSyncTriggerMsg(const librav::ReceiveBuffer *rbuf, const std::string &chan, const librav_lcm_msgs::SimSyncTrigger *msg)
{
    sync_trigger_ready_ = msg->trigger_ready;
}

void TrafficSimManager::UpdateSimState(double t)
{
    std::vector<VehicleState> states = vehicle_manager_->GetVehicleStatesAt(t);

    std::cout << "simulation time: " << t << std::endl;

    // broadcast simulation updates to LCM
    librav_lcm_msgs::VehicleEstimations ests_msg;

    ests_msg.vehicle_num = states.size();
    for (auto &state : states)
    {
        librav_lcm_msgs::VehicleState state_msg;

        auto pose = state.GetPose();
        auto fp = state.GetFootprint();
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

        ests_msg.estimations.push_back(state_msg);
    }
    data_link_->publish(CAV_COMMON_CHANNELS::VEHICLE_ESTIMATIONS_CHANNEL, &ests_msg);
}

void TrafficSimManager::RunSim(bool sync_mode)
{
    std::cout << "INFO: simulation started..." << std::endl;

    // simulation loop
    double t = config_.t0;
    while (t < config_.tf)
    {
        // label starting time of current sim iteration
        sim_stopwatch_.tic();

        // main simulation calculation happens here
        UpdateSimState(t);

        // handle LCM callbacks
        data_link_->handleTimeout(0);

        // check if sync_mode, enable this if planner runs too slow
        /*
         * In sync mode, simulation will wait for planner to react before starting next iteration.
         * Make sure the trigger signal is sent by some other node (such as the planner). 
         * Otherwise, the simulation will appear frozen.
         */
        if (sync_mode)
        {
            // wait until trigger signal is ready
            while (!sync_trigger_ready_)
            {
            };
            sync_trigger_ready_ = false;
        }

        // increase simulation time and wait for this iteration to end
        t += config_.dt;
        sim_stopwatch_.sleep_until_ms(config_.dt * 1000);
    }
}
