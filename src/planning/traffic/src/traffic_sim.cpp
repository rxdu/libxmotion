/* 
 * traffic_sim.cpp
 * 
 * Created on: Nov 19, 2017 15:06
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#include "traffic/traffic_sim.h"

using namespace librav;

TrafficSim::TrafficSim() : sim_duration_(0),
                           start_time_(0),
                           sim_t_(start_time_),
                           step_size_(0)
{
}

bool TrafficSim::UpdateTraffic()
{
    if(sim_t_ > sim_duration_ + start_time_)
        return true;

    // update states of traffic participants
    vehicle_.Update(sim_t_, step_size_);
    std::cout << "Vehicle position: " << vehicle_.GetPosition() << " ; vehicle velocity: " << vehicle_.GetVelocity() << std::endl; 

    sim_t_ += step_size_;

    return false;
}