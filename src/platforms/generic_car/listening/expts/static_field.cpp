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

int main() {
  const int32_t fsize_x = 100;
  const int32_t fsize_y = 100;
  auto cfield = std::make_shared<CollisionField>(fsize_x, fsize_y);

  FieldPlot surf_plot(fsize_x, fsize_y);
  surf_plot.SetCameraPosition(0, 50, 50);

  cfield->vehicle_field_->SetCarPosition(static_cast<int64_t>(50),
                                         static_cast<int64_t>(50));
  cfield->vehicle_field_->UpdateDistribution();
  cfield->CombineAllFields();

  // plot surface
  ScalarFieldMatrix mat = cfield->GenerateFieldMatrix(0, 1, 0, 1.5);
  Eigen::MatrixXd norm_mat_z = mat.z/mat.z.maxCoeff()*1.0;
  surf_plot.ShowField(mat.x, mat.y, norm_mat_z, false);

  return 0;
}