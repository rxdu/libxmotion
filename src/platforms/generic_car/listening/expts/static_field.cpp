#include <iostream>

// headers for lcm
#include "lcmtypes/librav.hpp"
#include <lcm/lcm-cpp.hpp>

#include "field/collision_field.hpp"
#include "stopwatch/stopwatch.h"
#include "traffic/traffic_sim.hpp"

#include "fastplot/field_plot.hpp"

using namespace librav;

#define LOOP_PERIOD 1500

int main()
{
  const int32_t fsize_x = 74;
  const int32_t fsize_y = 74*2;
  auto cfield = std::make_shared<CollisionField>(fsize_x, fsize_y);

  cfield->CreateAndAddVehicleField(1);
  cfield->CreateAndAddVehicleField(2);

  // setup vehicle field
  cfield->GetVehicleField(1)->SetCarPosition(19,50);
  cfield->GetVehicleField(2)->SetCarPosition(56,100);
  cfield->UpdateCollisionField();

  // plot surface
  FieldPlot surf_plot(fsize_x, fsize_y);
  surf_plot.SetCameraPosition(0, 50, 50);
  surf_plot.SetWrapScale(4.0);

  ScalarFieldMatrix mat = cfield->GenerateFieldMatrix(0, 1, 0, 1);
  Eigen::MatrixXd norm_mat_z = mat.z / mat.z.maxCoeff() * 1.0;
  surf_plot.ShowField(mat.x, mat.y, norm_mat_z, true);

  return 0;
}