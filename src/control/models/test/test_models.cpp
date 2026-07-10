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

// --- classical benchmark models (cart-pole / dynamic bicycle / quadrotor) ---

#include "xmnav/models/cartpole.hpp"
#include "xmnav/models/dynamic_bicycle.hpp"
#include "xmnav/models/quadrotor.hpp"

// unforced frictionless cart-pole conserves mechanical energy (the
// dynamics oracle for the Barto/Sutton formulation)
TEST(BenchmarkModelsTest, CartPoleEnergyConservedUnforced) {
  CartPoleModel model;
  CartPoleModel::State x;
  x << 0.0, 0.0, 2.5, 0.0;  // large swing, at rest
  const double e0 = model.Energy(x);
  const CartPoleModel::Control u = CartPoleModel::Control::Zero();
  x = Rk4Propagate(model, x, u, 0.0, 10.0, 1e-3);
  EXPECT_NEAR(model.Energy(x), e0, 1e-6 * std::abs(e0));
}

TEST(BenchmarkModelsTest, CartPoleUprightUnstableHangingStable) {
  CartPoleModel model;
  const CartPoleModel::Control u = CartPoleModel::Control::Zero();
  // small perturbation at the upright equilibrium grows
  CartPoleModel::State up;
  up << 0.0, 0.0, 1e-3, 0.0;
  up = Rk4Propagate(model, up, u, 0.0, 2.0, 1e-3);
  EXPECT_GT(std::abs(up(2)), 0.1);
  // ... and near the hanging equilibrium it oscillates, bounded
  CartPoleModel::State down;
  down << 0.0, 0.0, M_PI - 0.05, 0.0;
  down = Rk4Propagate(model, down, u, 0.0, 5.0, 1e-3);
  EXPECT_LT(std::abs(down(2) - M_PI), 0.10);
}

// constant speed + steering settles to Rajamani's steady-state yaw rate
TEST(BenchmarkModelsTest, DynamicBicycleSteadyStateCornering) {
  DynamicBicycleModel model;
  DynamicBicycleModel::State x = DynamicBicycleModel::State::Zero();
  x(3) = 20.0;  // m/s
  const double delta = 0.05;
  for (int i = 0; i < 5000; ++i) {
    DynamicBicycleModel::Control u;
    u << -x(4) * x(5), delta;  // ax cancels vy*r: vx held exactly
    x = model.Step(x, u, i, 1e-3);
  }
  EXPECT_NEAR(x(3), 20.0, 1e-9);
  EXPECT_NEAR(x(5), model.SteadyStateYawRate(20.0, delta),
              0.01 * std::abs(model.SteadyStateYawRate(20.0, delta)));
}

// at low speed the dynamic model degenerates to the kinematic bicycle
TEST(BenchmarkModelsTest, DynamicBicycleLowSpeedMatchesKinematic) {
  DynamicBicycleModel model;
  DynamicBicycleModel::State x = DynamicBicycleModel::State::Zero();
  x(3) = 2.0;
  const double delta = 0.05;
  for (int i = 0; i < 5000; ++i) {
    DynamicBicycleModel::Control u;
    u << -x(4) * x(5), delta;
    x = model.Step(x, u, i, 1e-3);
  }
  const double kinematic_r = 2.0 / (model.lf + model.lr) * std::tan(delta);
  EXPECT_NEAR(x(5), kinematic_r, 0.02 * kinematic_r);
}

TEST(BenchmarkModelsTest, QuadrotorHoverIsEquilibrium) {
  QuadrotorModel model;
  const auto x0 = QuadrotorModel::MakeState(
      Eigen::Vector3d(1.0, -2.0, 3.0), Eigen::Vector3d::Zero(),
      Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
  QuadrotorModel::Control u;
  u << model.HoverThrust(), 0.0, 0.0, 0.0;
  const auto x = Rk4Propagate(model, x0, u, 0.0, 1.0, 1e-3);
  EXPECT_LT((x - x0).norm(), 1e-9);
}

TEST(BenchmarkModelsTest, QuadrotorFreeFallAndYawSpin) {
  QuadrotorModel model;
  // free fall: z = -g t^2 / 2
  auto x = QuadrotorModel::MakeState(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
  x = Rk4Propagate(model, x, QuadrotorModel::Control::Zero(), 0.0, 1.0,
                   1e-3);
  EXPECT_NEAR(x(2), -0.5 * model.gravity, 1e-9);
  // constant yaw moment: omega_z = tau_z / Izz * t exactly (gyroscopic
  // term vanishes for pure z rotation); quaternion stays unit
  auto s = QuadrotorModel::MakeState(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
  QuadrotorModel::Control u;
  u << model.HoverThrust(), 0.0, 0.0, 1e-3;
  s = Rk4Propagate(model, s, u, 0.0, 2.0, 1e-4);
  EXPECT_NEAR(QuadrotorModel::BodyRates(s)(2),
              1e-3 / model.inertia_diag(2) * 2.0, 1e-6);
  EXPECT_NEAR(QuadrotorModel::Orientation(s).norm(), 1.0, 1e-9);
}

#include "xmnav/models/linearize.hpp"

// numeric linearization pins the known analytic Jacobian structure of
// the double integrator exactly (Deriv is linear: differences are exact)
TEST(BenchmarkModelsTest, NumericLinearizationMatchesLinearModel) {
  BicycleAccelModel model;  // nonlinear check: bicycle at a straight run
  BicycleAccelModel::State x0;
  x0 << 0.0, 0.0, 5.0, 0.0;
  const auto lin =
      LinearizeNumeric(model, x0, BicycleAccelModel::Control::Zero());
  // d(x_dot)/d(v) = cos(theta) = 1; d(y_dot)/d(theta) = v = 5
  EXPECT_NEAR(lin.A(0, 2), 1.0, 1e-6);
  EXPECT_NEAR(lin.A(1, 3), 5.0, 1e-6);
  // d(v_dot)/d(a) = 1; d(theta_dot)/d(delta) = v/L at delta = 0
  EXPECT_NEAR(lin.B(2, 0), 1.0, 1e-6);
  EXPECT_NEAR(lin.B(3, 1), 5.0 / model.wheelbase, 1e-5);
}

TEST(BenchmarkModelsTest, DynamicBicycleSlipAnglesConsistent) {
  DynamicBicycleModel model;
  DynamicBicycleModel::State x = DynamicBicycleModel::State::Zero();
  x(3) = 20.0;
  x(4) = 0.3;
  x(5) = 0.1;
  const auto alphas = model.SlipAngles(x, 0.05);
  EXPECT_NEAR(alphas(0), (0.3 + model.lf * 0.1) / 20.0 - 0.05, 1e-12);
  EXPECT_NEAR(alphas(1), (0.3 - model.lr * 0.1) / 20.0, 1e-12);
}
