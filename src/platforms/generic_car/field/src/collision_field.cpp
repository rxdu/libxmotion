/*
 * collision_field.cpp
 *
 * Created on: Nov 17, 2017 11:18
 * Description:
 *
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#include "field/collision_field.hpp"

using namespace librav;

CollisionField::CollisionField(int64_t size_x, int64_t size_y)
    : ScalarField(size_x, size_y)
{
}

void CollisionField::LoadPredefinedBasisPattern()
{
  // distribute the basis around the center of field
  // std::vector
  // for ()
    // auto basis = std::make_shared<ThreatBasis>(size_x_, size_y_);
}

void CollisionField::CreateAndAddThreatBasisField(int64_t x, int64_t y, int32_t id)
{
  threat_basis_fields_.emplace(std::make_pair(id, std::make_shared<ThreatBasis>(size_x_, size_y_)));
}

void CollisionField::AddThreatBasisField(int32_t id,
                                         std::shared_ptr<ThreatBasis> tfield)
{
  assert(tfield->SizeX() == this->size_x_ && tfield->SizeY() == this->size_y_);

  threat_basis_fields_.emplace(std::make_pair(id, tfield));
}

std::shared_ptr<ThreatBasis> CollisionField::GetThreatBasisField(int32_t id)
{
  assert(threat_basis_fields_.find(id) != threat_basis_fields_.end());

  return threat_basis_fields_[id];
}

void CollisionField::RemoveThreatBasisField(int32_t id)
{
  threat_basis_fields_.erase(id);
}

void CollisionField::UpdateCollisionField()
{
  for (int64_t i = 0; i < size_x_; ++i)
  {
    for (int64_t j = 0; j < size_y_; ++j)
    {
      double vehicle_val = 0;
      for (const auto &tfd : threat_basis_fields_)
        vehicle_val += tfd.second->GetValueAtCoordinate(i, j);
      SetValueAtCoordinate(i, j, vehicle_val);
    }
  }
}