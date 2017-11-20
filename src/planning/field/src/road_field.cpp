/* 
 * road_field.cpp
 * 
 * Created on: Nov 17, 2017 15:38
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#include "field/road_field.h"

using namespace librav;

RoadField::RoadField(int64_t size_x, int64_t size_y) : ScalarField(size_x, size_y),
                                                       drivable_area_potential_(0.01)
{
}

void RoadField::SetLocationDrivable(int64_t x, int64_t y)
{
    SetValueAtLocation(x, y, drivable_area_potential_);
}
