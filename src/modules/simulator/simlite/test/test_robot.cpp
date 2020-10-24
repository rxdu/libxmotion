/*
 * test_robot.cpp
 *
 * Created on: Oct 24, 2020 22:15
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#include <iostream>

#include "model/bicycle_model.hpp"
#include "simlite/robot.hpp"

using namespace ivnav;

int main(int argc, char *argv[]) {
  Robot<BicycleKinematics> robot;

  double t0 = 0;
  double tf = 10.0;
  double dt = 0.01;
  auto xt = robot.StepForward({0, 0, 0, 0}, {0.8, 0}, t0, tf, dt);
  std::cout << "x(t): " << xt[0] << " , " << xt[1] << " , " << xt[2] << " , "
            << xt[3] << std::endl;

  return 0;
}