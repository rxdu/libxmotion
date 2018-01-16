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
    : ScalarField(size_x, size_y),
      road_field_(std::make_shared<RoadField>(size_x, size_y)),
      vehicle_field_(std::make_shared<VehicleField>(size_x, size_y)) {
  // for (int64_t x = 26; x < 26 + 37 * 2; ++x)
  // {
  //     for(int64_t y = 0; y < 200; ++y)
  //         road_field_->SetLocationDrivable(x,y);
  // }

  // for (int64_t x = 100; x < 200; ++x)
  // {
  //     for(int64_t y = 100; y < 174; ++y)
  //         road_field_->SetLocationDrivable(x,y);
  // }
}

void CollisionField::AddVehicleField(int32_t id,
                                     std::shared_ptr<VehicleField> vfield) {
  vehicle_fields_.emplace(std::make_pair(id, vfield));
}

void CollisionField::RemoveVehicleField(int32_t id) {
  vehicle_fields_.erase(id);
}

void CollisionField::CombineAllFields() {
  for (int64_t i = 0; i < size_x_; ++i) {
    for (int64_t j = 0; j < size_y_; ++j) {
      // double road_val = road_field_->GetValueAtLocation(i,j);
      double vehicle_val = vehicle_field_->GetValueAtLocation(i, j);
      //   SetValueAtLocation(i, j, road_val + vehicle_val);
      SetValueAtLocation(i, j, vehicle_val);
    }
  }
}