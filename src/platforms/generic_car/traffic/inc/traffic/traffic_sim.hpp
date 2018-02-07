/* 
 * traffic_sim.hpp
 * 
 * Created on: Nov 19, 2017 15:06
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#ifndef TRAFFIC_SIM_HPP
#define TRAFFIC_SIM_HPP

#include <cstdint>

#include "traffic/vehicle_sim.hpp"
#include "stopwatch/stopwatch.h"

namespace librav
{

class TrafficSim
{
public:
    TrafficSim() = default;
    ~TrafficSim() = default;

    void SetDuration(double duration) { sim_duration_ = duration; };
    void SetStartTime(double start) { start_time_ = start; };
    void SetStepSize(double step) { step_size_ = step; };

    bool UpdateTraffic(int32_t period_ms = 0);
    VehicleSim vehicle_;

private:
    double sim_duration_ = 0;
    double start_time_ = 0;
    double sim_t_ = 0;
    double step_size_ = 0;

    stopwatch::StopWatch timer_;
};

}

#endif /* TRAFFIC_SIM_HPP */

