/* 
 * lattice_generator.hpp
 * 
 * Created on: Aug 07, 2018 00:26
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef LATTICE_GENERATOR_HPP
#define LATTICE_GENERATOR_HPP

#include <cmath>
#include <vector>
#include <utility>

#include "ascent/Ascent.h"
#include "ascent/Utility.h"

#include "model/bicycle_model.hpp"

namespace librav
{
class LatticeGenerator
{
public:
  LatticeGenerator() = default;

  void SetInitialState(double x, double y, double v, double theta);
  void GenerateControlSet();
  void RunSim(double t0, double tf, double dt);

private:
  asc::RK4 integrator_;

  double init_x_;
  double init_y_;
  double init_v_;
  double init_theta_;

  std::vector<BicycleKinematics::control_t> ctrl_set_;

  asc::state_t Propagate(asc::state_t init_state, BicycleKinematics::control_t u, double t0, double tf, double dt, int32_t idx);
};
} // namespace librav

#endif /* LATTICE_GENERATOR_HPP */
