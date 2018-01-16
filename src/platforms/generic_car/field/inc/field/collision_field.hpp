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
#include <unordered_map>

#include "field/road_field.hpp"
#include "field/scalar_field.hpp"
#include "field/vehicle_field.hpp"

namespace librav {

class CollisionField : public ScalarField {
public:
  CollisionField(int64_t size_x, int64_t size_y);

  void CombineAllFields();

  std::shared_ptr<RoadField> road_field_;
  std::shared_ptr<VehicleField> vehicle_field_;
  void AddVehicleField(int32_t id, std::shared_ptr<VehicleField> vfield);
  void RemoveVehicleField(int32_t id);

private:
  std::unordered_map<int32_t, std::shared_ptr<RoadField>> road_fields_;
  std::unordered_map<int32_t, std::shared_ptr<VehicleField>>
      vehicle_fields_;
};
}

#endif /* COLLISION_FIELD_HPP */
