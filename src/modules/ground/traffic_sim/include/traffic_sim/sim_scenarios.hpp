/* 
 * sim_scenarios.hpp
 * 
 * Created on: Dec 06, 2018 21:46
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef SIM_SCENARIOS_HPP
#define SIM_SCENARIOS_HPP

#include "traffic_sim/traffic_sim_config.hpp"

namespace librav
{
namespace SimScenario
{
// "intersection_single_lane_full"
TrafficSimConfig GenerateScenarioCase1();
TrafficSimConfig GenerateScenarioCase2();

// "urban_single_lane_loop_full"
TrafficSimConfig GenerateScenarioCase3();

// "one_way_merging_horizontal"
TrafficSimConfig GenerateScenarioCase4();
} // namespace SimScenario
} // namespace librav

#endif /* SIM_SCENARIOS_HPP */
