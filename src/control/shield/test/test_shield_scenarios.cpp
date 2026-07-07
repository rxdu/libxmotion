/*
 * test_shield_scenarios.cpp
 *
 * The acceptance scenarios of docs/control/safety_shield.md (S1-S4, S6 —
 * S5 arrives with the SRB shield), plus unit checks of the individual
 * layers: envelope math, ladder legality (refused events), barrier
 * minimality/constraint satisfaction.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "xmnav/shield/wheeled_shield.hpp"

using namespace xmotion;

namespace {

constexpr double kDt = 0.02;

WheeledShield::Config MakeConfig(bool with_obstacle = false) {
  WheeledShield::Config cfg;
  cfg.envelope.u_min << -0.8, -2.0;
  cfg.envelope.u_max << 0.8, 2.0;
  cfg.envelope.rate_limit << 2.0, 8.0;  // [m/s^2, rad/s^2]
  cfg.ladder.hold_timeout = 0.1;        // 5 ticks at 50 Hz
  cfg.ladder.stop_ramp_time = 0.2;      // 10 ticks
  cfg.state_staleness_max = 0.1;
  cfg.barrier.u_min = cfg.envelope.u_min;
  cfg.barrier.u_max = cfg.envelope.u_max;
  cfg.barrier.look_ahead = 0.15;
  cfg.barrier.alpha = 2.0;
  cfg.barrier.margin = 0.05;
  if (with_obstacle) {
    cfg.barrier.obstacles.push_back({Eigen::Vector2d(2.0, 0.0), 0.3});
  } else {
    cfg.enable_barrier = false;
  }
  return cfg;
}

const WheeledShield::State kOrigin = WheeledShield::State::Zero();

}  // namespace

// S1: a command spike is followed at the configured rate limit
TEST(ShieldScenarioTest, S1SpikeIsRateLimited) {
  WheeledShield shield(MakeConfig());
  const WheeledShield::Control spike(10.0, 0.0);  // beyond box and rate
  const auto u1 = shield.Filter(spike, kOrigin, 0.0, kDt);
  EXPECT_NEAR(u1(0), 2.0 * kDt, 1e-12);  // one rate step from zero
  EXPECT_TRUE(shield.LastReport().envelope_active);
  EXPECT_TRUE(shield.LastReport().modified);
  EXPECT_EQ(shield.mode(), ShieldMode::kNormal);
  // keeps ramping, then saturates at the box limit
  WheeledShield::Control u = u1;
  for (int i = 0; i < 100; ++i) u = shield.Filter(spike, kOrigin, 0.0, kDt);
  EXPECT_NEAR(u(0), 0.8, 1e-12);
}

// S2: NaN command -> hold, then controlled stop; early recovery resumes
TEST(ShieldScenarioTest, S2InvalidCommandDegradesThroughLadder) {
  WheeledShield shield(MakeConfig());
  const WheeledShield::Control cruise(0.5, 0.0);
  WheeledShield::Control u = WheeledShield::Control::Zero();
  for (int i = 0; i < 50; ++i) u = shield.Filter(cruise, kOrigin, 0.0, kDt);
  EXPECT_NEAR(u(0), 0.5, 1e-12);

  const WheeledShield::Control bad(std::nan(""), 0.0);
  // fault: holds the last safe command
  u = shield.Filter(bad, kOrigin, 0.0, kDt);
  EXPECT_EQ(shield.mode(), ShieldMode::kHold);
  EXPECT_NEAR(u(0), 0.5, 1e-12);
  EXPECT_FALSE(shield.LastReport().input_valid);
  // recovery within the timeout resumes normal operation
  u = shield.Filter(cruise, kOrigin, 0.0, kDt);
  EXPECT_EQ(shield.mode(), ShieldMode::kNormal);
  EXPECT_NEAR(u(0), 0.5, 1e-12);

  // persistent fault: hold expires -> ramp to zero -> latched stop
  double last_v = 0.5;
  bool saw_stopping = false;
  for (int i = 0; i < 30; ++i) {
    u = shield.Filter(bad, kOrigin, 0.0, kDt);
    if (shield.mode() == ShieldMode::kStopping) {
      saw_stopping = true;
      EXPECT_LE(u(0), last_v + 1e-12) << "ramp must be non-increasing";
      last_v = u(0);
    }
  }
  EXPECT_TRUE(saw_stopping);
  EXPECT_EQ(shield.mode(), ShieldMode::kStopped);
  EXPECT_DOUBLE_EQ(u(0), 0.0);
  // recovery alone does NOT re-arm; a guarded Reset does
  u = shield.Filter(cruise, kOrigin, 0.0, kDt);
  EXPECT_EQ(shield.mode(), ShieldMode::kStopped);
  EXPECT_TRUE(shield.Reset());
  u = shield.Filter(cruise, kOrigin, 0.0, kDt);
  EXPECT_EQ(shield.mode(), ShieldMode::kNormal);
}

// S3: the barrier attenuates the command approaching an obstacle and the
// closed loop never crosses the safety distance; far away it is untouched
TEST(ShieldScenarioTest, S3BarrierPreventsPenetration) {
  auto cfg = MakeConfig(/*with_obstacle=*/true);
  WheeledShield shield(cfg);
  const WheeledShield::Control full_speed(0.8, 0.0);

  // far away: minimal intervention (envelope ramp-up aside)
  WheeledShield::State x = kOrigin;  // obstacle 2 m ahead
  auto u = shield.Filter(full_speed, x, 0.0, kDt);
  EXPECT_FALSE(shield.LastReport().barrier_active);

  // drive at the obstacle with the shield in the loop
  double min_clearance = std::numeric_limits<double>::infinity();
  bool barrier_engaged = false;
  for (int i = 0; i < 600; ++i) {
    u = shield.Filter(full_speed, x, 0.0, kDt);
    x(0) += u(0) * std::cos(x(2)) * kDt;
    x(1) += u(0) * std::sin(x(2)) * kDt;
    x(2) += u(1) * kDt;
    const double clearance =
        (x.head<2>() - Eigen::Vector2d(2.0, 0.0)).norm() - 0.3;
    min_clearance = std::min(min_clearance, clearance);
    barrier_engaged |= shield.LastReport().barrier_active;
    ASSERT_EQ(shield.mode(), ShieldMode::kNormal) << "tick " << i;
  }
  EXPECT_TRUE(barrier_engaged);
  // never penetrates the margin band (allow the discrete-time epsilon)
  EXPECT_GT(min_clearance, 0.5 * cfg.barrier.margin);
}

// S4: a stale state estimate walks the same ladder as an invalid command
TEST(ShieldScenarioTest, S4StaleStateDegrades) {
  WheeledShield shield(MakeConfig());
  const WheeledShield::Control cruise(0.4, 0.0);
  for (int i = 0; i < 30; ++i) shield.Filter(cruise, kOrigin, 0.0, kDt);
  auto u = shield.Filter(cruise, kOrigin, /*state_age=*/0.5, kDt);
  EXPECT_EQ(shield.mode(), ShieldMode::kHold);
  EXPECT_GT(u(0), 0.0);  // holding, not dropping to zero instantly
  for (int i = 0; i < 30; ++i) u = shield.Filter(cruise, kOrigin, 0.5, kDt);
  EXPECT_EQ(shield.mode(), ShieldMode::kStopped);
  EXPECT_DOUBLE_EQ(u(0), 0.0);
}

// S6: e-stop from any mode; guarded reset re-arms
TEST(ShieldScenarioTest, S6EStopFromAnywhere) {
  WheeledShield shield(MakeConfig());
  const WheeledShield::Control cruise(0.5, 0.0);
  for (int i = 0; i < 30; ++i) shield.Filter(cruise, kOrigin, 0.0, kDt);
  EXPECT_EQ(shield.mode(), ShieldMode::kNormal);
  shield.TriggerEStop();
  auto u = shield.Filter(cruise, kOrigin, 0.0, kDt);
  EXPECT_EQ(shield.mode(), ShieldMode::kStopped);
  EXPECT_DOUBLE_EQ(u.norm(), 0.0);
  // reset with valid inputs -> normal
  EXPECT_TRUE(shield.Reset());
  shield.Filter(cruise, kOrigin, 0.0, kDt);
  EXPECT_EQ(shield.mode(), ShieldMode::kNormal);
  // e-stop mid-ramp too
  const WheeledShield::Control bad(std::nan(""), 0.0);
  for (int i = 0; i < 8; ++i) shield.Filter(bad, kOrigin, 0.0, kDt);
  shield.TriggerEStop();
  u = shield.Filter(bad, kOrigin, 0.0, kDt);
  EXPECT_EQ(shield.mode(), ShieldMode::kStopped);
  EXPECT_DOUBLE_EQ(u.norm(), 0.0);
  // reset is REFUSED while inputs are still invalid
  EXPECT_FALSE(shield.Reset());
}

// --- layer units ---

TEST(ShieldUnitTest, EnvelopeBoxAndRateIndependentPerChannel) {
  CommandEnvelope<2> env;
  env.u_min << -1.0, -2.0;
  env.u_max << 1.0, 2.0;
  env.rate_limit << 10.0, std::numeric_limits<double>::infinity();
  const Eigen::Vector2d prev(0.0, 0.0);
  bool clamped = false;
  // channel 0 rate-limited, channel 1 free until the box
  auto out = env.Apply({5.0, 5.0}, prev, 0.1, &clamped);
  EXPECT_TRUE(clamped);
  EXPECT_DOUBLE_EQ(out(0), 1.0);  // box binds before rate (10*0.1 = 1.0)
  EXPECT_DOUBLE_EQ(out(1), 2.0);
  out = env.Apply({0.5, 1.5}, prev, 0.1, &clamped);
  EXPECT_FALSE(clamped);
  EXPECT_DOUBLE_EQ(out(0), 0.5);
  EXPECT_DOUBLE_EQ(out(1), 1.5);
}

TEST(ShieldUnitTest, LadderRefusesIllegalEvents) {
  FallbackLadder ladder({0.1, 0.2});
  EXPECT_EQ(ladder.mode(), ShieldMode::kNormal);
  EXPECT_FALSE(ladder.Reset(true));  // no row (Normal, Reset)
  ladder.OnRecovered();              // no row (Normal, Recovered)
  EXPECT_EQ(ladder.mode(), ShieldMode::kNormal);
  ladder.OnFault();
  EXPECT_EQ(ladder.mode(), ShieldMode::kHold);
  ladder.OnFault();  // refused, stays
  EXPECT_EQ(ladder.mode(), ShieldMode::kHold);
}

TEST(ShieldUnitTest, BarrierIsMinimalAndSatisfiesConstraint) {
  DiffDriveBarrierFilter::Config cfg;
  cfg.obstacles.push_back({Eigen::Vector2d(1.0, 0.0), 0.2});
  cfg.u_min << -0.8, -2.0;
  cfg.u_max << 0.8, 2.0;
  DiffDriveBarrierFilter filter(cfg);
  ShieldReport report;
  // heading away from the obstacle: untouched
  const Eigen::Vector3d away(0.0, 0.0, M_PI);
  auto u = filter.Filter(away, {0.5, 0.0}, &report);
  EXPECT_FALSE(report.barrier_active);
  EXPECT_DOUBLE_EQ(u(0), 0.5);
  // close and head-on: modified, and the CBF row holds at the solution
  const Eigen::Vector3d close(0.5, 0.0, 0.0);
  u = filter.Filter(close, {0.8, 0.0}, &report);
  EXPECT_TRUE(report.barrier_active);
  EXPECT_LT(u(0), 0.8);
  // recompute h and a at the look-ahead point: a.u + alpha*h >= 0
  const Eigen::Vector2d p_l(0.5 + cfg.look_ahead, 0.0);
  const double safe =
      0.2 + cfg.margin + cfg.look_ahead;  // robot_radius = 0
  const Eigen::Vector2d d = p_l - Eigen::Vector2d(1.0, 0.0);
  const double h = d.squaredNorm() - safe * safe;
  Eigen::Matrix2d M;
  M << 1.0, 0.0, 0.0, cfg.look_ahead;  // theta = 0
  const Eigen::Vector2d a = 2.0 * M.transpose() * d;
  EXPECT_GE(a.dot(u) + cfg.alpha * h, -1e-9);
}
