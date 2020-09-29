/* 
 * vehicle_manager.hpp
 * 
 * Created on: Dec 06, 2018 07:54
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef VEHICLE_MANAGER_HPP
#define VEHICLE_MANAGER_HPP

#include <vector>
#include <memory>

#include "cav_common/vehicle_state.hpp"
#include "traffic_map/traffic_map.hpp"
#include "traffic_sim/vehicle_info.hpp"
#include "traffic_sim/vehicle_motion.hpp"

namespace librav
{
class VehicleManager
{
  public:
    VehicleManager(std::shared_ptr<TrafficMap> tmap);

    void AddVehicles(const std::vector<VehicleInfo> &vehs);
    std::vector<VehicleState> GetVehicleStatesAt(double t);

  private:
    std::shared_ptr<TrafficMap> traffic_map_;
    // std::vector<VehicleInfo> surrounding_vehicles_;
    std::vector<std::pair<VehicleInfo, VehicleMotion>> surrounding_vehicles_;
};
} // namespace librav

#endif /* VEHICLE_MANAGER_HPP */
