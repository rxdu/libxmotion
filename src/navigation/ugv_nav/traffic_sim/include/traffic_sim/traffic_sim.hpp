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

#include <memory>
#include <cstdint>

#include "traffic_sim/vehicle_sim.hpp"
#include "road_map/road_map.hpp"

#include "stopwatch/stopwatch.h"

namespace librav
{

class TrafficSim
{
  public:
    TrafficSim(std::shared_ptr<RoadMap> map);
    ~TrafficSim() = default;

    void SetDuration(double duration) { sim_duration_ = duration; };
    void SetStartTime(double start) { start_time_ = start; };
    void SetStepSize(double step) { step_size_ = step; };

    bool UpdateTraffic(int32_t period_ms = 0);
    VehicleSim vehicle_;

  private:
    std::shared_ptr<RoadMap> road_map_;

    double sim_duration_ = 0;
    double start_time_ = 0;
    double sim_t_ = 0;
    double step_size_ = 0;

    stopwatch::StopWatch timer_;
};

} // namespace librav

#endif /* TRAFFIC_SIM_HPP */
