/* 
 * field_utils.cpp
 * 
 * Created on: Nov 17, 2017 15:50
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#include "field/field_utils.h"

using namespace librav;

std::shared_ptr<RoadField> FieldUtils::CreateTestRoadField()
{
    auto field = std::make_shared<RoadField>(5, 5);

    for (int64_t i = 2; i < 5; i++)
    {
        for(int64_t j = 0; j < 5; j++)
            field->SetLocationDrivable(i,j);
    }

    return field;
}

std::shared_ptr<RoadField> FieldUtils::CreateDemoRoadField()
{
    auto field = std::make_shared<RoadField>(200, 200);

    for (int64_t x = 26; x < 26 + 37 * 2; ++x)
    {
        for(int64_t y = 0; y < 200; ++y)
            field->SetLocationDrivable(x,y);
    }

    for (int64_t x = 100; x < 200; ++x)
    {
        for(int64_t y = 100; y < 174; ++y)
            field->SetLocationDrivable(x,y);
    }

    return field;
}

std::shared_ptr<VehicleField> FieldUtils::CreateTestVehicleField()
{
    auto field = std::make_shared<VehicleField>(50, 50);

    field->SetCarPosition(24,24);
    field->UpdateDistribution();

    return field;
}