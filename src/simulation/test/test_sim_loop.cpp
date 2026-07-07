/*
 * test_sim_loop.cpp
 *
 * The headless simulation contract: determinism under a seed, noise
 * injection statistics, model-mismatch wiring, logging and export.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>

#include <cmath>
#include <sstream>

#include "xmnav/mppi/critics.hpp"
#include "xmnav/mppi/models/diff_drive.hpp"
#include "xmnav/mppi/mppi.hpp"
#include "xmnav/sim/sim_loop.hpp"

using namespace xmotion;

TEST(SimLoopTest, DeterministicUnderSeed) {
  SimLoop<DiffDriveModel>::Config cfg;
  cfg.dt = 0.05;
  cfg.steps = 100;
  cfg.seed = 42;
  cfg.process_noise << 0.01, 0.01, 0.005;
  cfg.measurement_noise << 0.02, 0.02, 0.01;

  auto agent = [](const DiffDriveModel::State& z, int) {
    return DiffDriveModel::Control(0.5, 0.3 * std::sin(z(2)));
  };

  SimLoop<DiffDriveModel> a(DiffDriveModel{}, cfg);
  SimLoop<DiffDriveModel> b(DiffDriveModel{}, cfg);
  const auto xa = a.Run(DiffDriveModel::State::Zero(), agent);
  const auto xb = b.Run(DiffDriveModel::State::Zero(), agent);
  EXPECT_TRUE(xa.isApprox(xb));
}

TEST(SimLoopTest, NoiseFreeMatchesDirectIntegration) {
  SimLoop<DiffDriveModel>::Config cfg;
  cfg.dt = 0.05;
  cfg.steps = 50;
  auto agent = [](const DiffDriveModel::State&, int) {
    return DiffDriveModel::Control(0.4, 0.2);
  };
  SimLoop<DiffDriveModel> sim(DiffDriveModel{}, cfg);
  const auto x_sim = sim.Run(DiffDriveModel::State::Zero(), agent);

  DiffDriveModel m;
  DiffDriveModel::State x = DiffDriveModel::State::Zero();
  for (int t = 0; t < 50; ++t) {
    x = m.Step(x, {0.4, 0.2}, t, 0.05);
  }
  EXPECT_TRUE(x_sim.isApprox(x));
}

TEST(SimLoopTest, LogRecordsAndExports) {
  SimLoop<DiffDriveModel>::Config cfg;
  cfg.dt = 0.1;
  cfg.steps = 10;
  SimLoop<DiffDriveModel> sim(DiffDriveModel{}, cfg);
  SimLog log;
  sim.Run(DiffDriveModel::State::Zero(),
          [](const DiffDriveModel::State&, int) {
            return DiffDriveModel::Control(1.0, 0.0);
          },
          &log);

  ASSERT_EQ(log.size(), 10);
  EXPECT_NEAR(log.state(9)(0), 1.0, 1e-9);  // 1 m/s for 1 s
  EXPECT_DOUBLE_EQ(log.control(0)(0), 1.0);

  std::ostringstream os;
  log.WriteJsonl(os);
  const std::string s = os.str();
  EXPECT_NE(s.find("\"states\":[["), std::string::npos);
  EXPECT_EQ(std::count(s.begin(), s.end(), '{'),
            std::count(s.begin(), s.end(), '}'));
}

TEST(SimLoopTest, ClosedLoopWithMppiUnderModelMismatch) {
  // controller plans on the nominal model; the plant runs with process
  // noise — the loop still reaches the goal (the whole point of feedback)
  Se2GoalCost cost;
  cost.goal << 2.0, 0.5, 0.0;
  using Controller = Mppi<DiffDriveModel, Se2GoalCost>;
  Controller::Params p;
  p.num_samples = 256;
  p.horizon_steps = 30;
  p.dt = 0.05;
  p.lambda = 0.3;
  p.sigma << 0.3, 0.8;
  p.u_min << -0.8, -2.0;
  p.u_max << 0.8, 2.0;
  Controller mppi(DiffDriveModel{}, cost, p);

  SimLoop<DiffDriveModel>::Config cfg;
  cfg.dt = 0.05;
  cfg.steps = 300;
  cfg.seed = 9;
  cfg.process_noise << 0.002, 0.002, 0.004;

  SimLoop<DiffDriveModel> sim(DiffDriveModel{}, cfg);
  const auto x_end =
      sim.Run(DiffDriveModel::State::Zero(),
              [&](const DiffDriveModel::State& z, int) {
                mppi.Plan(z);
                return mppi.Command();
              });

  EXPECT_NEAR(x_end(0), 2.0, 0.25);
  EXPECT_NEAR(x_end(1), 0.5, 0.25);
}
