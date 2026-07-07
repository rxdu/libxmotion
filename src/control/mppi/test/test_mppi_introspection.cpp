/*
 * test_mppi_introspection.cpp
 *
 * The introspection contract: captured candidates are exact re-computations
 * of what the optimizer scored (the "accurate" requirement), selection and
 * weights are coherent, the sink emits well-formed records, and disabled
 * introspection stays inert.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>

#include <sstream>

#include "xmnav/mppi/critics.hpp"
#include "xmnav/mppi/introspection_io.hpp"
#include "xmnav/models/diff_drive.hpp"
#include "xmnav/mppi/mppi.hpp"

using namespace xmotion;

namespace {
using Controller = Mppi<DiffDriveModel, Se2GoalCost>;

Controller Make(bool introspect) {
  Se2GoalCost cost;
  cost.goal << 2.0, 1.0, 0.0;
  Controller::Params p;
  p.num_samples = 512;
  p.horizon_steps = 25;
  p.dt = 0.05;
  p.lambda = 0.2;
  p.normalize_cost_spread = true;
  p.control_cost_decoupling = 1.0;  // gamma = 0: cost is purely state cost
  p.sigma << 0.3, 0.8;
  p.seed = 5;
  Controller c(DiffDriveModel{}, cost, p);
  if (introspect) c.EnableIntrospection(8, 8);
  return c;
}
}  // namespace

TEST(MppiIntrospectionTest, CandidateCostsAreExactRecomputations) {
  auto mppi = Make(true);
  mppi.Plan(Controller::State::Zero());
  const auto& snap = mppi.LastSnapshot();
  ASSERT_GE(snap.candidates.size(), 8u);

  Se2GoalCost cost;
  cost.goal << 2.0, 1.0, 0.0;
  for (const auto& cand : snap.candidates) {
    double recomputed = 0.0;
    for (Eigen::Index t = 0; t < cand.states.rows(); ++t) {
      recomputed += cost.StageCost(cand.states.row(t).transpose(),
                                   cand.controls.row(t).transpose(),
                                   static_cast<int>(t));
    }
    recomputed += cost.TerminalCost(
        cand.states.row(cand.states.rows() - 1).transpose());
    EXPECT_NEAR(recomputed, cand.cost, 1e-9 * std::max(1.0, cand.cost));
  }
}

TEST(MppiIntrospectionTest, SelectionAndWeightsCoherent) {
  auto mppi = Make(true);
  mppi.Plan(Controller::State::Zero());
  const auto& snap = mppi.LastSnapshot();

  // first candidate is the best sample overall
  EXPECT_DOUBLE_EQ(snap.candidates.front().cost, mppi.LastBestCost());
  double max_w = 0.0;
  for (const auto& c : snap.candidates) max_w = std::max(max_w, c.weight);
  EXPECT_DOUBLE_EQ(snap.candidates.front().weight, max_w);

  EXPECT_EQ(snap.nominal_states.rows(), 25);
  EXPECT_GT(snap.effective_sample_size, 1.5);
  EXPECT_EQ(snap.plan_index, 1u);
}

TEST(MppiIntrospectionTest, JsonlRecordWellFormed) {
  auto mppi = Make(true);
  mppi.Plan(Controller::State::Zero());
  std::ostringstream os;
  AppendSnapshotJsonl(os, mppi.LastSnapshot());
  const std::string s = os.str();

  EXPECT_EQ(s.back(), '\n');
  EXPECT_NE(s.find("\"plan\":1"), std::string::npos);
  EXPECT_NE(s.find("\"candidates\":["), std::string::npos);
  EXPECT_NE(s.find("\"nominal_states\":[["), std::string::npos);
  // balanced brackets (cheap structural sanity)
  EXPECT_EQ(std::count(s.begin(), s.end(), '['),
            std::count(s.begin(), s.end(), ']'));
  EXPECT_EQ(std::count(s.begin(), s.end(), '{'),
            std::count(s.begin(), s.end(), '}'));
}

TEST(MppiIntrospectionTest, DisabledStaysInert) {
  auto mppi = Make(false);
  mppi.Plan(Controller::State::Zero());
  EXPECT_TRUE(mppi.LastSnapshot().candidates.empty());
  EXPECT_EQ(mppi.LastSnapshot().plan_index, 0u);
}
