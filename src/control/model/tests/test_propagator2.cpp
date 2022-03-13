/*
 * test_propagator.cpp
 *
 * Created on: Mar 21, 2018 15:28
 * Description:
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include <iostream>

#include <boost/numeric/odeint.hpp>

#include "model/bicycle_model.hpp"

using namespace robotnav;
using namespace boost::numeric::odeint;

int main() {
  double t0 = 0;
  double tf = 10;
  double dt = 0.01;

  BicycleKinematics::state_type x = {0.0, 0.0, 0.0, 0.0};
  BicycleKinematics model({0.8, 0});

  boost::numeric::odeint::integrate_const(
      boost::numeric::odeint::runge_kutta4<BicycleKinematics::state_type>(),
      model, x, 0.0, 10.0, 0.01);

  //   runge_kutta4<std::vector<double>> rk4;
  //   double t = t0;
  //   for (size_t i = 0; i < 1000; ++i, t += dt) {
  //     rk4.do_step(model, x, t, dt);
  //   }

  std::cout << "final state: " << x[0] << " , " << x[1] << std::endl;

  return 0;
}