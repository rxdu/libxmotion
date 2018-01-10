#include <iostream>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

#include "field/field_base.h"
#include "field/vehicle_field.h"
#include "field/field_utils.h"
#include "field/collision_field.h"

using namespace librav;

int main()
{
    auto test_vehicle_field = FieldUtils::CreateTestVehicleField();

    librav_lcm_msgs::ScalarField_t msg = test_vehicle_field->GenerateScalarFieldMsg();

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