/*
 * test_models.cpp
 *
 * Tests for the shared models module: the accel-bicycle + RK4 pair that
 * replaced model/BicycleKinematics + boost::odeint SystemPropagator
 * (checked against closed-form solutions), and Step-concept sanity for
 * the planar models. The MPPI/CUDA suites exercise the models far more
 * heavily; these pin the module's own contracts.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>

#include <cmath>

#include "xmnav/models/ackermann.hpp"
#include "xmnav/models/bicycle_accel.hpp"
#include "xmnav/models/diff_drive.hpp"
#include "xmnav/models/double_integrator.hpp"
#include "xmnav/models/rk4.hpp"

using namespace xmotion;

// straight line, constant acceleration: closed form x = v0 t + a t^2 / 2
TEST(ModelsTest, BicycleAccelStraightLineMatchesClosedForm) {
  BicycleAccelModel model;
  BicycleAccelModel::State x0;
  x0 << 0.0, 0.0, 8.0, 0.0;
  BicycleAccelModel::Control u;
  u << 1.5, 0.0;  // accelerate, no steering
  const auto x = Rk4Propagate(model, x0, u, 0.0, 2.0, 0.01);
  EXPECT_NEAR(x(0), 8.0 * 2.0 + 0.5 * 1.5 * 4.0, 1e-9);  // 19.0
  EXPECT_NEAR(x(1), 0.0, 1e-12);
  EXPECT_NEAR(x(2), 11.0, 1e-12);
  EXPECT_NEAR(x(3), 0.0, 1e-12);
}

// constant speed + steering: theta(t) = v/L tan(delta) t, circle of radius
// R = L / tan(delta); RK4 at dt = 0.01 must be very close to closed form
TEST(ModelsTest, BicycleAccelConstantSteeringTracksCircle) {
  BicycleAccelModel model;  // wheelbase 2.4
  BicycleAccelModel::State x0;
  x0 << 0.0, 0.0, 5.0, 0.0;
  BicycleAccelModel::Control u;
  u << 0.0, 0.2;
  const double tf = 3.0;
  const auto x = Rk4Propagate(model, x0, u, 0.0, tf, 0.01);
  const double omega = 5.0 / model.wheelbase * std::tan(0.2);
  const double R = 5.0 / omega;
  const double theta = omega * tf;
  EXPECT_NEAR(x(3), theta, 1e-9);
  EXPECT_NEAR(x(0), R * std::sin(theta), 1e-6);
  EXPECT_NEAR(x(1), R * (1.0 - std::cos(theta)), 1e-6);
}

// the final RK4 step is shortened to land exactly on tf
TEST(ModelsTest, Rk4LandsExactlyOnTf) {
  BicycleAccelModel model;
  BicycleAccelModel::State x0;
  x0 << 0.0, 0.0, 1.0, 0.0;
  BicycleAccelModel::Control u = BicycleAccelModel::Control::Zero();
  // 0.7 / 0.2 = 3.5 steps: the last one must be half-length
  const auto x = Rk4Propagate(model, x0, u, 0.0, 0.7, 0.2);
  EXPECT_NEAR(x(0), 0.7, 1e-12);
}

// Euler Step at small dt approaches the RK4 trajectory (Step concept)
TEST(ModelsTest, BicycleAccelStepConvergesToRk4) {
  BicycleAccelModel model;
  BicycleAccelModel::State x_euler;
  x_euler << 0.0, 0.0, 5.0, 0.0;
  BicycleAccelModel::Control u;
  u << 0.5, 0.1;
  const double dt = 1e-4;
  for (int t = 0; t < 10000; ++t) x_euler = model.Step(x_euler, u, t, dt);
  const auto x_rk4 = Rk4Propagate(model, {0.0, 0.0, 5.0, 0.0}, u, 0.0, 1.0,
                                  0.01);
  EXPECT_LT((x_euler - x_rk4).norm(), 1e-3);
}

// planar Step-concept models: one sane step each (heavily exercised by
// the MPPI suites; this pins the module boundary)
TEST(ModelsTest, PlanarModelsStepSanity) {
  DiffDriveModel dd;
  auto xd = dd.Step(DiffDriveModel::State::Zero(), {1.0, 0.0}, 0, 0.1);
  EXPECT_NEAR(xd(0), 0.1, 1e-12);

  AckermannModel ack;
  auto xa = ack.Step(AckermannModel::State::Zero(), {1.0, 0.1}, 0, 0.1);
  EXPECT_NEAR(xa(0), 0.1, 1e-12);
  EXPECT_NEAR(xa(2), 1.0 / ack.wheelbase * std::tan(0.1) * 0.1, 1e-12);

  DoubleIntegratorModel di;
  DoubleIntegratorModel::Control ui;
  ui << 1.0;
  auto xi = di.Step(DoubleIntegratorModel::State::Zero(), ui, 0, 0.1);
  EXPECT_TRUE(xi.allFinite());
}
