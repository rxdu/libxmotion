/* 
 * traffic_sim_config.hpp
 * 
 * Created on: Dec 05, 2018 21:36
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef TRAFFIC_SIM_CONFIG_HPP
#define TRAFFIC_SIM_CONFIG_HPP

#include <cstdint>
#include <string>
#include <vector>

#include "traffic_sim/vehicle_info.hpp"

namespace ivnav
{
struct TrafficSimConfig
{
    /* sim timing */
    const double t0 = 0.0;
    double tf;
    double dt = 1;

    /* estimation broadcasting timing */
    // default: broadcast after every simulation update
    int32_t dtk = 1;

    /* map */
    std::string map;

    /* initial traffic configuration */
    VehicleInfo ego_vehicle;
    std::vector<VehicleInfo> surrounding_vehicles;
};
} // namespace ivnav

#endif /* TRAFFIC_SIM_CONFIG_HPP */
