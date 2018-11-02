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

#include "road_map/road_map.hpp"
#include "reachability/markov_occupancy.hpp"

namespace librav
{
class CollisionThreat
{
  public:
    CollisionThreat(std::shared_ptr<TrafficChannel> chn);

    std::shared_ptr<TrafficChannel> traffic_chn_;
};
} // namespace librav

#endif /* COLLISION_THREAT_HPP */
