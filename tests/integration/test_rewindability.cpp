/*
 * test_rewindability.cpp
 *
 * Rewindability regression (the second "adopt now" item of
 * docs/research/ihmc-open-robotics-software.md §6, after IHMC's
 * DRCFlatGroundRewindabilityTest): run a scenario, snapshot the loop and
 * the algorithm stack mid-run by COPY, resume the copies from that tick,
 * and bitwise-diff the recorded tail against the original run.
 *
 * What this pins that the units cannot: every component in the loop is a
 * self-contained value — copying it captures ALL state that influences
 * future outputs (RNG streams, warm-start buffers, rate-limit memory,
 * FSM state). Any hidden state, shallow copy, or nondeterminism shows up
 * as a bitwise mismatch in the resumed tail. Bitwise is the contract on
 * the CPU path (mppi.hpp documents determinism for any thread count); do
 * not weaken these asserts to isApprox.
 *
 * Snapshot boundary: the SimLoop observer fires after the process-noise
 * draw of its tick, i.e. at a clean RNG boundary — copying at tick k-1
 * captures the generator exactly where tick k's draws begin (contract
 * documented on SimLoop::RunFrom).
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>

#include <cmath>
#include <optional>

#include "xmnav/models/diff_drive.hpp"
#include "xmnav/mppi/critics.hpp"
#include "xmnav/mppi/mppi.hpp"
#include "xmnav/shield/wheeled_shield.hpp"
#include "xmnav/sim/sim_loop.hpp"

using namespace xmotion;

namespace {

#if !defined(NDEBUG) || defined(__SANITIZE_ADDRESS__) || \
    defined(__SANITIZE_THREAD__)
constexpr int kSamples = 128;
#else
constexpr int kSamples = 256;
#endif

constexpr double kDt = 0.05;

// bitwise equality of log A rows [k, k+len) against log B rows [0, len)
void ExpectTailBitwiseEqual(const SimLog &a, const SimLog &b, int k,
                            int len) {
  ASSERT_EQ(b.size(), len);
  EXPECT_TRUE((a.states().middleRows(k, len).array() ==
               b.states().array())
                  .all());
  EXPECT_TRUE((a.measurements().middleRows(k, len).array() ==
               b.measurements().array())
                  .all());
  EXPECT_TRUE((a.controls().middleRows(k, len).array() ==
               b.controls().array())
                  .all());
  for (int i = 0; i < len; ++i) {
    EXPECT_EQ(a.time(k + i), b.time(i)) << "row " << i;
  }
}

}  // namespace

TEST(RewindabilityTest, SimLoopRewind) {
  // plant-only determinism of the loop machinery: open-loop t-dependent
  // agent, both noise channels ON so the snapshot must carry the RNG
  constexpr int kSteps = 400;
  constexpr int kRewindTick = 200;

  SimLoop<DiffDriveModel>::Config cfg;
  cfg.dt = kDt;
  cfg.steps = kSteps;
  cfg.seed = 7;
  cfg.process_noise << 0.01, 0.01, 0.005;
  cfg.measurement_noise << 0.02, 0.02, 0.01;

  auto agent = [](const DiffDriveModel::State &, int t) {
    return DiffDriveModel::Control(0.5, 0.4 * std::sin(0.1 * t));
  };

  SimLoop<DiffDriveModel> sim(DiffDriveModel{}, cfg);
  std::optional<SimLoop<DiffDriveModel>> sim_snapshot;
  DiffDriveModel::State x_snapshot = DiffDriveModel::State::Zero();

  SimLog log_a;
  sim.Run(DiffDriveModel::State::Zero(), agent, &log_a,
          [&](const DiffDriveModel::State &x, const DiffDriveModel::Control &,
              int t) {
            if (t == kRewindTick - 1) {
              sim_snapshot.emplace(sim);
              x_snapshot = x;
            }
          });
  ASSERT_TRUE(sim_snapshot.has_value());

  constexpr int kTail = kSteps - kRewindTick;
  SimLog log_b;
  sim_snapshot->RunFrom(kRewindTick, kTail, x_snapshot, agent, &log_b);
  ExpectTailBitwiseEqual(log_a, log_b, kRewindTick, kTail);
}

TEST(RewindabilityTest, ComposedPipelineRewind) {
  // the real regression: Mppi (warm-start buffers + sampler RNG + thread
  // pool) and WheeledShield (FSM + rate-limit memory) must be rewindable
  // by copy alongside the loop
  constexpr int kSteps = 300;
  constexpr int kRewindTick = 150;

  Se2GoalCost cost;
  cost.goal << 3.0, 1.0, 0.0;
  using Controller = Mppi<DiffDriveModel, Se2GoalCost>;
  Controller::Params p;
  p.num_samples = kSamples;
  p.horizon_steps = 30;
  p.dt = kDt;
  p.lambda = 0.3;
  p.sigma << 0.3, 0.8;
  p.u_min << -0.8, -2.0;
  p.u_max << 0.8, 2.0;
  p.normalize_cost_spread = true;
  Controller mppi(DiffDriveModel{}, cost, p);

  // obstacle-free shield config (same rationale as the allocation tier):
  // pins the envelope + ladder passthrough path
  WheeledShield::Config sc;
  sc.envelope.u_min = p.u_min;
  sc.envelope.u_max = p.u_max;
  sc.envelope.rate_limit << 3.0, 10.0;
  sc.enable_barrier = false;
  WheeledShield shield(sc);

  SimLoop<DiffDriveModel>::Config cfg;
  cfg.dt = kDt;
  cfg.steps = kSteps;
  cfg.seed = 21;
  cfg.process_noise << 0.002, 0.002, 0.004;
  cfg.measurement_noise << 0.01, 0.01, 0.005;

  auto make_agent = [&](Controller &c, WheeledShield &s) {
    return [&c, &s](const DiffDriveModel::State &z, int) {
      c.Plan(z);
      return s.Filter(c.Command(), z, /*state_age=*/0.0, kDt);
    };
  };

  SimLoop<DiffDriveModel> sim(DiffDriveModel{}, cfg);
  std::optional<SimLoop<DiffDriveModel>> sim_snapshot;
  std::optional<Controller> mppi_snapshot;
  std::optional<WheeledShield> shield_snapshot;
  DiffDriveModel::State x_snapshot = DiffDriveModel::State::Zero();

  SimLog log_a;
  sim.Run(DiffDriveModel::State::Zero(), make_agent(mppi, shield), &log_a,
          [&](const DiffDriveModel::State &x, const DiffDriveModel::Control &,
              int t) {
            if (t == kRewindTick - 1) {
              sim_snapshot.emplace(sim);
              mppi_snapshot.emplace(mppi);
              shield_snapshot.emplace(shield);
              x_snapshot = x;
            }
          });
  ASSERT_TRUE(sim_snapshot.has_value());
  ASSERT_EQ(shield.mode(), ShieldMode::kNormal);

  constexpr int kTail = kSteps - kRewindTick;
  SimLog log_b;
  sim_snapshot->RunFrom(kRewindTick, kTail, x_snapshot,
                        make_agent(*mppi_snapshot, *shield_snapshot),
                        &log_b);
  ExpectTailBitwiseEqual(log_a, log_b, kRewindTick, kTail);
}
