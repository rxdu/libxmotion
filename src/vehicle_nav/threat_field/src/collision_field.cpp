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

CollisionField::CollisionField(double xmin, double xmax, double ymin, double ymax)
    : xmin_(xmin), xmax_(xmax), ymin_(ymin), ymax_(ymax)
{
}

void CollisionField::SetSize(double xmin, double xmax, double ymin, double ymax)
{
  xmin_ = xmin;
  xmax_ = xmax;
  ymin_ = ymin;
  ymax_ = ymax;
}

void CollisionField::AddTrafficParticipant(int32_t id, std::shared_ptr<TrafficParticipant> tfield)
{
  traffic_participants_.emplace(std::make_pair(id, tfield));
}

std::shared_ptr<TrafficParticipant> CollisionField::GetTrafficParticipant(int32_t id)
{
  assert(traffic_participants_.find(id) != traffic_participants_.end());

  return traffic_participants_[id];
}

void CollisionField::RemoveTrafficParticipant(int32_t id)
{
  traffic_participants_.erase(id);
}

double CollisionField::operator()(double x, double y)
{
  double val = 0;
  for (auto tp : traffic_participants_)
    val += tp.second->threat_func(x, y);
  return val;
}
