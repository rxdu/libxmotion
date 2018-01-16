/* 
 * field_utils.hpp
 * 
 * Created on: Nov 17, 2017 15:48
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef FIELD_UTILS_HPP
#define FIELD_UTILS_HPP

#include <memory>

#include "field/road_field.hpp"
#include "field/vehicle_field.hpp"

namespace librav
{

namespace FieldUtils
{
    std::shared_ptr<RoadField> CreateTestRoadField();
    std::shared_ptr<RoadField> CreateDemoRoadField();

    std::shared_ptr<VehicleField> CreateTestVehicleField();
}

}

#endif /* FIELD_UTILS_HPP */
