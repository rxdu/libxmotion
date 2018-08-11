/*
 * collision_field.hpp
 *
 * Created on: Nov 17, 2017 11:18
 * Description: This is the field for planning, which may consist of multiple
 *          layers of sub-fields describing different types of collisions
 *
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef COLLISION_FIELD_HPP
#define COLLISION_FIELD_HPP

#include <memory>
#include <vector>
#include <cstdint>
#include <unordered_map>

#include "threat_field/traffic_participant.hpp"
#include "threat_field/threat_distribution.hpp"

namespace librav
{
class CollisionField
{
public:
  CollisionField(int64_t size_x, int64_t size_y);

  // typedef TrafficParticipant<GaussianPositionVelocityThreat> TrafficParticipantType;
  // typedef TrafficParticipant<BiasedGaussianThreat> TrafficParticipantType;
  
  void AddTrafficParticipant(int32_t id, std::shared_ptr<TrafficParticipant> participant);
  std::shared_ptr<TrafficParticipant> GetTrafficParticipant(int32_t id);
  void RemoveTrafficParticipant(int32_t id);

  void UpdateCollisionField();

private:
  std::unordered_map<int32_t, std::shared_ptr<TrafficParticipant>> traffic_participants_;
};
} // namespace librav

#endif /* COLLISION_FIELD_HPP */
