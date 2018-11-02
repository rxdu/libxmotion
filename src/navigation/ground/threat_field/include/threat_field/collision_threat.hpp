/* 
 * collision_threat.hpp
 * 
 * Created on: Nov 02, 2018 01:37
 * Description: collision threat \psi_i for vehicle_i
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef COLLISION_THREAT_HPP
#define COLLISION_THREAT_HPP

#include <memory>

#include "traffic_map/traffic_map.hpp"
#include "threat_field/vehicle_estimation.hpp"
#include "reachability/markov_occupancy.hpp"

namespace librav
{
class CollisionThreat
{
  public:
    CollisionThreat(VehicleEstimation est, std::shared_ptr<TrafficChannel> chn);

    VehicleEstimation vehicle_est_;
    std::shared_ptr<TrafficChannel> traffic_chn_;

  private:
    // Markov model covarage: 4m * 20 = 80m, 
    MarkovOccupancy<10, 5> occupancy_;
    void SetupPredictionModel();
};
} // namespace librav

#endif /* COLLISION_THREAT_HPP */
