/*
 * collision_field.cpp
 *
 * Created on: Nov 17, 2017 11:18
 * Description:
 *
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#include "threat_field/collision_field.hpp"

#include <cmath>
#include <iostream>

#include <Eigen/Dense>

using namespace librav;

CollisionField::CollisionField(int64_t size_x, int64_t size_y)
    : ScalarField(size_x, size_y)
{
  lane_threat_matrix_.resize(size_y, size_x);
  lane_threat_matrix_ =  Eigen::MatrixXd::Zero(size_y, size_x);
}

void CollisionField::AddLaneConstraints(int32_t id, std::shared_ptr<LaneConstraint> lane_constr)
{
  assert(lane_constr->SizeX() == this->size_x_ && lane_constr->SizeY() == this->size_y_);

  lane_constraints_.emplace(std::make_pair(id, lane_constr));

  UpdateLaneConstraintMatrix();
}

void CollisionField::RemoveLaneConstraints(int32_t id)
{
  lane_constraints_.erase(id);

  UpdateLaneConstraintMatrix();
}

void CollisionField::AddTrafficParticipant(int32_t id, std::shared_ptr<TrafficParticipantType> tfield)
{
  assert(tfield->SizeX() == this->size_x_ && tfield->SizeY() == this->size_y_);

  traffic_participants_.emplace(std::make_pair(id, tfield));
}

std::shared_ptr<TrafficParticipantType> CollisionField::GetTrafficParticipant(int32_t id)
{
  assert(traffic_participants_.find(id) != traffic_participants_.end());

  return traffic_participants_[id];
}

void CollisionField::RemoveTrafficParticipant(int32_t id)
{
  traffic_participants_.erase(id);
}

void CollisionField::UpdateCollisionField()
{
  for (int64_t i = 0; i < size_x_; ++i)
    for (int64_t j = 0; j < size_y_; ++j)
    {
      double threat_val = 0; //lane_threat_matrix_(i, j);
      for (const auto &tfd : traffic_participants_)
        threat_val += tfd.second->GetValueAtCoordinate(i, j);
      SetValueAtCoordinate(i, j, threat_val);
    }

  collision_threat_matrix_ = GenerateFieldMatrix(0, 0.1, 0, 0.1, true).z + lane_threat_matrix_*0.1;
}

void CollisionField::UpdateLaneConstraintMatrix()
{
  for (int64_t i = 0; i < size_x_; ++i)
    for (int64_t j = 0; j < size_y_; ++j)
    {
      double threat_val = 0;
      for (const auto &lc : lane_constraints_)
        threat_val += lc.second->GetValueAtCoordinate(i, j);
      lane_threat_matrix_(j, i) = threat_val;
    }
}