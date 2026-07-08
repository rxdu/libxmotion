/*
 * test_pid.cpp
 *
 * Tests for the classical linear feedback family: PID production
 * behaviors (derivative-on-measurement, D filtering, both anti-windup
 * strategies, defensive boundary), DLQR synthesis against closed-loop
 * stability, and multi-dimensional state feedback with integral action
 * rejecting a constant disturbance.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>

#include <cmath>

#include "xmnav/pid/dlqr.hpp"
#include "xmnav/pid/pid_controller.hpp"
#include "xmnav/pid/state_feedback.hpp"

using namespace xmotion;

namespace {
constexpr double kDt = 0.01;
}

TEST(PidTest, ProportionalOnlyMatchesHandComputation) {
  PidController::Config cfg;
  cfg.kp = 2.0;
  PidController pid(cfg);
  EXPECT_DOUBLE_EQ(pid.Update(1.0, 0.25, kDt), 1.5);
}

// derivative on measurement: a setpoint step must NOT kick the D term
TEST(PidTest, NoDerivativeKickOnSetpointStep) {
  PidController::Config cfg;
  cfg.kp = 1.0;
  cfg.kd = 1.0;
  PidController pid(cfg);
  pid.Update(0.0, 0.0, kDt);
  const double u = pid.Update(10.0, 0.0, kDt);  // setpoint jumps
  EXPECT_DOUBLE_EQ(u, 10.0);                    // pure P, D stays zero
  // a measurement jump DOES produce a derivative response
  const double u2 = pid.Update(10.0, 1.0, kDt);
  EXPECT_LT(u2, 9.0 - 50.0);  // kp*9 - kd*(1.0/0.01)
}

TEST(PidTest, DerivativeFilterBoundsTheResponse) {
  PidController::Config raw_cfg;
  raw_cfg.kd = 1.0;
  PidController::Config filt_cfg = raw_cfg;
  filt_cfg.d_filter_tau = 0.1;
  PidController raw(raw_cfg), filt(filt_cfg);
  raw.Update(0.0, 0.0, kDt);
  filt.Update(0.0, 0.0, kDt);
  const double u_raw = raw.Update(0.0, 1.0, kDt);
  const double u_filt = filt.Update(0.0, 1.0, kDt);
  EXPECT_LT(std::abs(u_filt), std::abs(u_raw) * 0.2);
}

// with conditional integration, a long saturated phase must not wind the
// integral up: on reference return, the output leaves saturation promptly
TEST(PidTest, ConditionalIntegrationPreventsWindup) {
  PidController::Config cfg;
  cfg.kp = 1.0;
  cfg.ki = 10.0;
  cfg.u_min = -1.0;
  cfg.u_max = 1.0;
  PidController pid(cfg);
  for (int i = 0; i < 1000; ++i) pid.Update(100.0, 0.0, kDt);  // saturated
  EXPECT_LE(std::abs(pid.integral()), 1.0 + 1e-9);
  const double u = pid.Update(0.0, 0.0, kDt);  // reference returns
  EXPECT_LT(u, 1.0);                           // out of saturation at once
}

TEST(PidTest, BackCalculationBleedsTheIntegral) {
  PidController::Config cfg;
  cfg.ki = 10.0;
  cfg.u_max = 1.0;
  cfg.anti_windup = PidController::AntiWindup::kBackCalculation;
  cfg.back_calc_gain = 20.0;
  PidController pid(cfg);
  for (int i = 0; i < 1000; ++i) pid.Update(100.0, 0.0, kDt);
  // the integral settles near the saturation limit instead of 100*ki*t
  EXPECT_LT(pid.integral(), 60.0);
  EXPECT_GT(pid.integral(), 1.0);
}

// closed loop on a first-order plant: converges to the reference
TEST(PidTest, ClosedLoopFirstOrderPlantConverges) {
  PidController::Config cfg;
  cfg.kp = 4.0;
  cfg.ki = 8.0;
  cfg.u_min = -5.0;
  cfg.u_max = 5.0;
  PidController pid(cfg);
  double y = 0.0;
  for (int i = 0; i < 2000; ++i) {
    const double u = pid.Update(1.0, y, kDt);
    y += (-y + u) * kDt;  // tau = 1 plant
  }
  EXPECT_NEAR(y, 1.0, 1e-3);
}

TEST(PidTest, DefensiveBoundaryLeavesStateUntouched) {
  PidController::Config cfg;
  cfg.kp = 1.0;
  cfg.ki = 1.0;
  PidController pid(cfg);
  const double u0 = pid.Update(1.0, 0.0, kDt);
  const double i0 = pid.integral();
  EXPECT_DOUBLE_EQ(pid.Update(std::nan(""), 0.0, kDt), u0);
  EXPECT_DOUBLE_EQ(pid.Update(1.0, 0.0, -1.0), u0);
  EXPECT_DOUBLE_EQ(pid.integral(), i0);
}

// --- DLQR + state feedback ---

namespace {
// discrete double integrator
const Eigen::Matrix2d kA = (Eigen::Matrix2d() << 1.0, kDt, 0.0, 1.0).finished();
const Eigen::Vector2d kB{0.5 * kDt * kDt, kDt};
}  // namespace

TEST(DlqrTest, DoubleIntegratorGainStabilizes) {
  const auto result = SolveDlqr<2, 1>(kA, kB, Eigen::Matrix2d::Identity(),
                                      Eigen::Matrix<double, 1, 1>::Identity());
  ASSERT_TRUE(result.converged);
  const Eigen::Matrix2d Acl = kA - kB * result.K;
  EXPECT_LT(std::abs(Acl.eigenvalues()(0)), 1.0);
  EXPECT_LT(std::abs(Acl.eigenvalues()(1)), 1.0);
  // P is the cost-to-go: symmetric positive definite
  EXPECT_NEAR((result.P - result.P.transpose()).cwiseAbs().maxCoeff(), 0.0,
              1e-9);
  EXPECT_GT(result.P.eigenvalues().real().minCoeff(), 0.0);
}

TEST(StateFeedbackTest, DlqrGainRegulatesToReference) {
  const auto lqr = SolveDlqr<2, 1>(kA, kB, Eigen::Matrix2d::Identity(),
                                   Eigen::Matrix<double, 1, 1>::Identity());
  ASSERT_TRUE(lqr.converged);
  StateFeedbackController<2, 1>::Config cfg;
  cfg.K = lqr.K;
  StateFeedbackController<2, 1> ctrl(cfg);
  Eigen::Vector2d x{2.0, 0.0};
  const Eigen::Vector2d x_ref{0.5, 0.0};
  for (int i = 0; i < 4000; ++i) {
    const auto u = ctrl.Update(x_ref, x, kDt);
    x = kA * x + kB * u(0);
  }
  EXPECT_NEAR(x(0), 0.5, 1e-6);
  EXPECT_NEAR(x(1), 0.0, 1e-6);
}

// a constant input disturbance leaves an offset without integral action
// and none with it (the point of the LQI augmentation)
TEST(StateFeedbackTest, IntegralActionRejectsConstantDisturbance) {
  const auto lqr = SolveDlqr<2, 1>(kA, kB, Eigen::Matrix2d::Identity(),
                                   Eigen::Matrix<double, 1, 1>::Identity());
  ASSERT_TRUE(lqr.converged);
  const double disturbance = 0.5;
  const Eigen::Vector2d x_ref{0.0, 0.0};

  StateFeedbackController<2, 1>::Config plain_cfg;
  plain_cfg.K = lqr.K;
  StateFeedbackController<2, 1> plain(plain_cfg);
  Eigen::Vector2d x{0.0, 0.0};
  for (int i = 0; i < 6000; ++i) {
    const auto u = plain.Update(x_ref, x, kDt);
    x = kA * x + kB * (u(0) + disturbance);
  }
  const double offset_without_integral = std::abs(x(0));
  EXPECT_GT(offset_without_integral, 1e-3);

  // design the integral gain properly: DLQR on the integrator-augmented
  // system x_a = [x; z], z_{k+1} = z_k + dt C (x_ref - x_k)
  Eigen::Matrix3d Aa = Eigen::Matrix3d::Zero();
  Aa.topLeftCorner<2, 2>() = kA;
  Aa(2, 0) = -kDt;  // z integrates -x(0) when x_ref = 0
  Aa(2, 2) = 1.0;
  Eigen::Vector3d Ba;
  Ba << kB, 0.0;
  Eigen::Matrix3d Qa = Eigen::Matrix3d::Identity();
  Qa(2, 2) = 4.0;  // weight on the integrated error
  const auto lqi_gain = SolveDlqr<3, 1>(
      Aa, Ba, Qa, Eigen::Matrix<double, 1, 1>::Identity());
  ASSERT_TRUE(lqi_gain.converged);

  StateFeedbackController<2, 1, 1>::Config lqi_cfg;
  lqi_cfg.K = lqi_gain.K.leftCols<2>();  // u = K e + Ki z, e = x_ref - x
  lqi_cfg.C << 1.0, 0.0;                 // integrate the position error
  lqi_cfg.Ki << -lqi_gain.K(0, 2);       // u = -Ka x_a  ->  Ki = -Ka_z
  StateFeedbackController<2, 1, 1> lqi(lqi_cfg);
  x.setZero();
  for (int i = 0; i < 6000; ++i) {
    const auto u = lqi.Update(x_ref, x, kDt);
    x = kA * x + kB * (u(0) + disturbance);
  }
  EXPECT_NEAR(x(0), 0.0, 1e-4);
}

TEST(StateFeedbackTest, SaturationFreezesTheIntegrator) {
  StateFeedbackController<2, 1, 1>::Config cfg;
  cfg.K << 1.0, 1.0;
  cfg.C << 1.0, 0.0;
  cfg.Ki << 1.0;
  cfg.u_min << -1.0;
  cfg.u_max << 1.0;
  StateFeedbackController<2, 1, 1> ctrl(cfg);
  const Eigen::Vector2d x_ref{100.0, 0.0};
  const Eigen::Vector2d x{0.0, 0.0};
  for (int i = 0; i < 1000; ++i) ctrl.Update(x_ref, x, kDt);
  // deep saturation from the first tick: the integrator never advances
  EXPECT_DOUBLE_EQ(ctrl.integrator()(0), 0.0);
}
