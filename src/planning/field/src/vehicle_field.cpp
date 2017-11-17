/* 
 * vehicle_field.cpp
 * 
 * Created on: Nov 17, 2017 17:48
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#include "field/vehicle_field.h"

using namespace librav;

VehicleField::VehicleField(int64_t size_x, int64_t size_y) : ScalarField<double>(size_x, size_y),
                                                       drivable_area_potential_(1.5)
{
}

void VehicleField::SetLocationDrivable(int64_t x, int64_t y)
{
    SetValueAtLocation(x, y, drivable_area_potential_);
}
