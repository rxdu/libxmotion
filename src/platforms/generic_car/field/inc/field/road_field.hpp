/* 
 * road_field.hpp
 * 
 * Created on: Nov 17, 2017 15:32
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef ROAD_FIELD_HPP
#define ROAD_FIELD_HPP

#include "common/librav_types.hpp"
#include "field/scalar_field.hpp"
#include "lcmtypes/librav.hpp"

namespace librav
{

class RoadField : public ScalarField
{
  public:
    RoadField() = delete;
    RoadField(int64_t size_x, int64_t size_y);
    ~RoadField() = default;

    void SetLocationDrivable(int64_t x, int64_t y);

  private:
    const double drivable_area_potential_;
};
}

#endif /* ROAD_FIELD_HPP */
