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

void CollisionField::CreateAndAddVehicleField(int32_t id)
{
    vehicle_fields_.emplace(std::make_pair(id, std::make_shared<VehicleField>(size_x_, size_y_)));
}

void CollisionField::AddVehicleField(int32_t id,
                                     std::shared_ptr<VehicleField> vfield)
{
  assert(vfield->SizeX() == this->size_x_ && vfield->SizeY() == this->size_y_);

  vehicle_fields_.emplace(std::make_pair(id, vfield));
}

std::shared_ptr<VehicleField> CollisionField::GetVehicleField(int32_t id)
{
  assert(vehicle_fields_.find(id) != vehicle_fields_.end());

  return vehicle_fields_[id];
}

void CollisionField::RemoveVehicleField(int32_t id)
{
  vehicle_fields_.erase(id);
}

void CollisionField::UpdateCollisionField()
{
  // update distribution of subfields first
  for (const auto &vf : vehicle_fields_)
    vf.second->UpdateDistribution();

  for (int64_t i = 0; i < size_x_; ++i)
  {
    for (int64_t j = 0; j < size_y_; ++j)
    {
      double vehicle_val = 0;
      for (const auto &vf : vehicle_fields_)
        vehicle_val += vf.second->GetValueAtCoordinate(i, j);
      SetValueAtCoordinate(i, j, vehicle_val);
    }
  }
}

void CollisionField::CombineChildFieldsWithNoUpdate()
{
  for (int64_t i = 0; i < size_x_; ++i)
  {
    for (int64_t j = 0; j < size_y_; ++j)
    {
      double vehicle_val = 0;
      for (const auto &vf : vehicle_fields_)
        vehicle_val += vf.second->GetValueAtCoordinate(i, j);
      SetValueAtCoordinate(i, j, vehicle_val);
    }
  }
}