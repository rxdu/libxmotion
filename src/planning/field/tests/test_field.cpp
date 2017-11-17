#include <iostream>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

#include "field/field_base.h"
#include "field/road_field.h"
#include "field/field_utils.h"
#include "field/collision_field.h"

using namespace librav;

int main()
{
    FieldBase<double> field(2, 3);

    std::cout << "field created" << std::endl;

    //field.PrintField();

    // RoadField road_field(2,3);
    // road_field.PrintField();

    // auto test_road_field = FieldUtils::CreateTestRoadField();
    // test_road_field->PrintField();
    auto test_road_field = FieldUtils::CreateDemoRoadField();

    librav_lcm_msgs::ScalarField_t msg = test_road_field->GenerateScalarFieldMsg();

    std::cout << "msg generated" << std::endl;

    // set up network first
    std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();
    if (!lcm->good())
    {
        std::cerr << "ERROR: Failed to initialize LCM." << std::endl;
        return -1;
    }

    lcm->publish("ScalarField", &msg);

    return 0;
}