#include <iostream>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

#include "field/field_base.hpp"
#include "field/vehicle_field.hpp"
#include "field/field_utils.hpp"
#include "field/collision_field.hpp"

#include "fastplot/surface_plot.hpp"

using namespace librav;

int main()
{
    auto test_vehicle_field = FieldUtils::CreateTestVehicleField();

    ScalarFieldMatrix mat = test_vehicle_field->GenerateFieldMatrix(0, 1, 0, 1.5);
    // plot surface
    SurfacePlot splot;
    splot.ShowSurface(mat.x,mat.y,mat.z, true);
    // splot.SaveSurfaceToFile(x, y, z, "test_surf.png");
    // librav_lcm_msgs::ScalarField_t msg = test_vehicle_field->GenerateScalarFieldMsg();

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