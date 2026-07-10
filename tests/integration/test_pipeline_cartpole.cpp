/*
 * test_pipeline_cartpole.cpp
 *
 * Integration scenario: cart-pole swing-up and catch — the classical
 * controller-handover benchmark.
 *
 *   MPPI (global, nonlinear swing-up) --near upright--> AlignOutput()
 *   --> DLQR state feedback (local catch + hold)
 *
 * The DLQR gain is synthesized in-test from a NUMERIC linearization of
 * the model at the upright equilibrium (finite differences on Deriv),
 * so the test also pins the model/linearization/dlqr/state-feedback
 * tool chain end to end. The plant integrates with RK4 while MPPI plans
 * with its own Euler rollouts — a deliberate integration-method
 * mismatch, stated here per the test-conditions rule.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>

#include <cmath>

#include "xmnav/models/cartpole.hpp"
#include "xmnav/models/linearize.hpp"
#include "xmnav/models/rk4.hpp"
#include "xmnav/mppi/mppi.hpp"
#include "xmnav/pid/dlqr.hpp"
#include "xmnav/pid/state_feedback.hpp"

using namespace xmotion;

namespace {

#if !defined(NDEBUG) || defined(__SANITIZE_ADDRESS__) || \
    defined(__SANITIZE_THREAD__)
constexpr int kSamples = 128;
#else
constexpr int kSamples = 512;
#endif

constexpr double kDt = 0.02;

double WrapAngle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

// swing-up cost: pole up (wrap-free via cos), cart near the origin, mild
// damping on the rates
struct SwingUpCost {
  using State = CartPoleModel::State;
  using Control = CartPoleModel::Control;
  double StageCost(const State &x, const Control & /*u*/, int /*t*/) const {
    return 10.0 * (1.0 - std::cos(x(2))) + 0.05 * x(0) * x(0) +
           0.05 * x(1) * x(1) + 0.01 * x(3) * x(3);
  }
  double TerminalCost(const State &x) const {
    return 10.0 * StageCost(x, Control::Zero(), 0);
  }
};

}  // namespace

TEST(PipelineIntegrationTest, CartPoleSwingUpThenDlqrCatch) {
  CartPoleModel model;

  // --- global controller: MPPI swing-up ---
  using Controller = Mppi<CartPoleModel, SwingUpCost>;
  Controller::Params p;
  p.num_samples = kSamples;
  p.horizon_steps = 60;  // 1.2 s lookahead
  p.dt = kDt;
  p.lambda = 0.3;
  p.sigma << 6.0;
  p.u_min << -10.0;
  p.u_max << 10.0;
  p.normalize_cost_spread = true;
  Controller mppi(CartPoleModel{}, SwingUpCost{}, p);

  // --- local controller: DLQR catch, gain synthesized in-test ---
  const auto lin = LinearizeNumeric(model, CartPoleModel::State::Zero(),
                                    CartPoleModel::Control::Zero());
  const Eigen::Matrix4d Ad = Eigen::Matrix4d::Identity() + lin.A * kDt;
  const Eigen::Vector4d Bd = lin.B * kDt;
  Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
  Q.diagonal() << 10.0, 1.0, 100.0, 10.0;
  const auto lqr = SolveDlqr<4, 1>(Ad, Bd, Q,
                                   Eigen::Matrix<double, 1, 1>::Identity());
  ASSERT_TRUE(lqr.converged);
  StateFeedbackController<4, 1>::Config fc;
  fc.K = lqr.K;
  fc.u_min << -10.0;
  fc.u_max << 10.0;
  StateFeedbackController<4, 1> catcher(fc);

  // --- closed loop: hanging start, RK4 plant ---
  CartPoleModel::State x;
  x << 0.0, 0.0, M_PI, 0.0;
  bool caught = false;
  int catch_tick = -1;
  const int total_ticks = 500;  // 10 s
  for (int t = 0; t < total_ticks; ++t) {
    CartPoleModel::Control u;
    const double theta = WrapAngle(x(2));
    if (!caught && std::abs(theta) < 0.35 && std::abs(x(3)) < 2.0) {
      caught = true;
      catch_tick = t;
      // bumpless handover at the current operating point
      CartPoleModel::State x_fb = x;
      x_fb(2) = theta;
      catcher.AlignOutput(mppi.Command(), CartPoleModel::State::Zero(),
                          x_fb);
    }
    if (caught) {
      CartPoleModel::State x_fb = x;
      x_fb(2) = theta;  // feedback operates on the wrapped angle
      u = catcher.Update(CartPoleModel::State::Zero(), x_fb, kDt);
    } else {
      mppi.Plan(x);
      u = mppi.Command();
    }
    x = Rk4Propagate(model, x, u, 0.0, kDt, kDt / 4.0);
  }

  ASSERT_TRUE(caught) << "swing-up never reached the catch region";
  EXPECT_LT(catch_tick, 350) << "swing-up too slow (7 s budget)";
  // held upright at the end: pole up, cart bounded, rates settled
  EXPECT_LT(std::abs(WrapAngle(x(2))), 0.05);
  EXPECT_LT(std::abs(x(3)), 0.5);
  EXPECT_LT(std::abs(x(0)), 2.0);
}
