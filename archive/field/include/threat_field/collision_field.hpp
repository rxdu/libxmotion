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
#include <unordered_map>
#include <vector>
#include <cstdint>

#include "threat_field/lane_constraint.hpp"
#include "threat_field/traffic_participant.hpp"
#include "threat_field/threat_distribution.hpp"

namespace librav
{
typedef TrafficParticipant<GaussianPositionVelocityThreat> TrafficParticipantType;

class CollisionField : public ScalarField
{
public:
  CollisionField(int64_t size_x, int64_t size_y);

  void AddLaneConstraints(int32_t id, std::shared_ptr<LaneConstraint> lane_constr);
  void RemoveLaneConstraints(int32_t id);

  void AddTrafficParticipant(int32_t id, std::shared_ptr<TrafficParticipantType> participant);
  std::shared_ptr<TrafficParticipantType> GetTrafficParticipant(int32_t id);
  void RemoveTrafficParticipant(int32_t id);

  void UpdateCollisionField();

  Eigen::MatrixXd collision_threat_matrix_;
  Eigen::MatrixXd lane_threat_matrix_;

private:
  std::unordered_map<int32_t, std::shared_ptr<TrafficParticipantType>> traffic_participants_;
  std::unordered_map<int32_t, std::shared_ptr<LaneConstraint>> lane_constraints_;

  void UpdateLaneConstraintMatrix();
};
}

#endif /* COLLISION_FIELD_HPP */
