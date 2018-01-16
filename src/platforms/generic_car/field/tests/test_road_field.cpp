#include <iostream>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

#include "field/field_base.hpp"
#include "field/road_field.hpp"
#include "field/field_utils.hpp"
#include "field/collision_field.hpp"

#include "fastplot/field_plot.hpp"

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
    ScalarFieldMatrix mat = test_road_field->GenerateFieldMatrix(0, 1, 0, 1.5);

    // plot surface
    FieldPlot fplot(2,3);
    fplot.ShowField(mat.x, mat.y, mat.z, false);

    // librav_lcm_msgs::ScalarField_t msg = test_road_field->GenerateScalarFieldMsg();

    // std::cout << "msg generated" << std::endl;

    // // set up network first
    // std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();
    // if (!lcm->good())
    // {
    //     std::cerr << "ERROR: Failed to initialize LCM." << std::endl;
    //     return -1;
    // }

    // lcm->publish("ScalarField", &msg);

    return 0;
}