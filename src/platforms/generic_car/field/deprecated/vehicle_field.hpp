/* 
 * vehicle_field.hpp
 * 
 * Created on: Nov 17, 2017 17:49
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef VEHICLE_FIELD_HPP
#define VEHICLE_FIELD_HPP

#include "lcmtypes/librav.hpp"

#include "field/scalar_field.hpp"
#include "common/librav_types.hpp"

namespace librav
{

class VehicleField : public ScalarField
{
  public:
    VehicleField() = delete;
    VehicleField(int64_t size_x, int64_t size_y);
    ~VehicleField() = default;

    void SetCarPosition(int64_t x, int64_t y);
    void SetCarVelocity()
    void UpdateDistribution();

  private:
    Position2Di com_pos_;
    Position2Di com_vel_;
};
}

#endif /* VEHICLE_FIELD_HPP */
