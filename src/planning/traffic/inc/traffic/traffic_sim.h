/* 
 * traffic_sim.h
 * 
 * Created on: Nov 19, 2017 15:06
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#ifndef TRAFFIC_SIM_H
#define TRAFFIC_SIM_H

#include "traffic/vehicle_sim.h"

namespace librav
{

class TrafficSim
{
public:
    TrafficSim();
    ~TrafficSim()=default;

    void SetDuration(double duration) { sim_duration_ = duration; };
    void SetStartTime(double start) { start_time_ = start; };
    void SetStepSize(double step) { step_size_ = step; };

    bool UpdateTraffic();
    VehicleSim vehicle_;

private:
    double sim_duration_;
    double start_time_;
    double sim_t_;
    double step_size_;
};

}

#endif /* TRAFFIC_SIM_H */

