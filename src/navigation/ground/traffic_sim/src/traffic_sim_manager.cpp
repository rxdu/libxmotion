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

#include "cav_common/cav_datalink.hpp"

using namespace librav;

TrafficSimManager::TrafficSimManager(TrafficSimConfig config) : config_(config)
{
    // setup communication link
    data_link_ = std::make_shared<LCMLink>();
    if (!data_link_->IsGood())
        std::cerr << "ERROR: Failed to initialize LCM." << std::endl;
    data_link_ready_ = true;

    data_link_->subscribe(CAV_SIM_CHANNELS::SIM_SYNC_TRIGGER_CHANNEL, &TrafficSimManager::HandleLCMMessage_SyncTrigger, this);
}

bool TrafficSimManager::ValidateSimConfig()
{
    // MapLoader loader("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_full.osm");

    // check datalink status
    if (!data_link_ready_)
        return false;

    return true;
}

void TrafficSimManager::HandleLCMMessage_SyncTrigger(const lcm_link::ReceiveBuffer *rbuf, const std::string &chan, const librav_lcm_msgs::SimSyncTrigger *msg)
{
    sync_trigger_ready_ = msg->trigger_ready;
}

void TrafficSimManager::RunSim(bool sync_mode)
{
    std::cout << "INFO: simulation started..." << std::endl;

    // simulation loop
    double t = config_.t0;
    while (t < config_.tf)
    {
        sim_stopwatch_.tic();

        std::cout << "simulation time: " << t << std::endl;
        // sim.UpdateTraffic(LOOP_PERIOD);

        // broadcast simulation updates to LCM
        librav_lcm_msgs::VehicleState state;
        state.position[0] = 1;
        state.position[1] = 2;
        state.theta = M_PI * 30.0 / 180.0;

        librav_lcm_msgs::VehicleEstimations ests_msg;
        ests_msg.vehicle_num = 1;
        ests_msg.estimations.push_back(state);

        data_link_->publish(CAV_COMMON_CHANNELS::VEHICLE_ESTIMATIONS_CHANNEL, &ests_msg);

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
