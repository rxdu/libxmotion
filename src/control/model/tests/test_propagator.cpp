/*
 * test_propagator.cpp
 *
 * Created on: Mar 21, 2018 15:28
 * Description:
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include <iostream>

#include "model/bicycle_model.hpp"
#include "model/system_propagator.hpp"

using namespace robosw;

class Lorenz {
 public:
  using control_type = double;
  using state_type = std::vector<double>;

  Lorenz(control_type u = 0){};

  void operator()(state_type &x, state_type &dxdt, double t) {
    dxdt[0] = sigma * (x[1] - x[0]);
    dxdt[1] = R * x[0] - x[1] - x[0] * x[2];
    dxdt[2] = x[0] * x[1] - b * x[2];
  }

  const double sigma = 10.0;
  const double R = 28.0;
  const double b = 8.0 / 3.0;
};

int main() {
  //   SystemPropagator<BicycleKinematics, BicycleKinematics::control_type>
  //       propagator;

  //   BicycleKinematics::state_type state =
  //       propagator.Propagate({0.0, 8.0}, {0.1, 0}, 0, 10, 0.01);

  //   std::cout << "final state: " << state[0] << " , " << state[1] <<
  //   std::endl;

  SystemPropagator<Lorenz, Lorenz::control_type> propagator;

  BicycleKinematics::state_type state =
      propagator.Propagate({10.0, 10.0, 10.0}, 0, 0, 10, 0.01);

  std::cout << "final state: " << state[0] << " , " << state[1] << " , "
            << state[2] << std::endl;
  return 0;
}