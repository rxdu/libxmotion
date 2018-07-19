#include <iostream>

// headers for lcm
#include "lcmtypes/librav.hpp"
#include <lcm/lcm-cpp.hpp>

#include "field/collision_field.hpp"
#include "stopwatch/stopwatch.h"
#include "traffic/traffic_sim.hpp"

#include "fastplot/field_plot.hpp"

using namespace librav;

#define LOOP_PERIOD 500

int main()
{
  const int32_t fsize_x = 74;
  const int32_t fsize_y = 250;
  auto cfield = std::make_shared<CollisionField>(fsize_x, fsize_y);

  cfield->CreateAndAddVehicleField(1);
  cfield->CreateAndAddVehicleField(2);

  // setup vehicle field
  cfield->GetVehicleField(1)->SetCarPosition(19, 50);
  cfield->GetVehicleField(2)->SetCarPosition(56, 100);
  cfield->UpdateCollisionField();

  // setup field plot
  FieldPlot field_plot(fsize_x, fsize_y);
  field_plot.SetCameraPosition(0, 50, 50);
  field_plot.SetWrapScale(4.0);

  stopwatch::StopWatch timer;
  for (int i = 0; i < 30; i++)
  {
    std::cout << "loop: " << i << std::endl;
    timer.tic();

    cfield->GetVehicleField(1)->SetCarPosition(19, static_cast<int64_t>(80-i*2.0));
    cfield->GetVehicleField(2)->SetCarPosition(56, static_cast<int64_t>(50+i*5.0));
    cfield->UpdateCollisionField();

    // plot surface
    ScalarFieldMatrix mat = cfield->GenerateFieldMatrix(0, 1.0, 0, 1.0);
    Eigen::MatrixXd norm_mat_z = mat.z / mat.z.maxCoeff() * 1.0;
    field_plot.ShowFieldFrame(mat.x, mat.y, mat.z / mat.z.maxCoeff() * 1.0, true);

    timer.sleep_until_ms(LOOP_PERIOD);
  }

  return 0;
}