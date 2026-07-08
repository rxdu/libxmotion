/*
 * test_mppi_draw.cpp
 *
 * Smoke test: an introspection snapshot from a real controller renders
 * onto a canvas without incident.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>

#include "xmnav/mppi/critics.hpp"
#include "xmnav/models/diff_drive.hpp"
#include "xmnav/mppi/mppi.hpp"
#include "xmnav/viz/mppi_draw.hpp"

using namespace xmotion;

TEST(MppiDrawTest, SnapshotRendersOntoCanvas) {
  Se2GoalCost cost;
  cost.goal << 2.0, 1.0, 0.0;
  using Controller = Mppi<DiffDriveModel, Se2GoalCost>;
  Controller::Params p;
  p.num_samples = 256;
  p.horizon_steps = 30;
  p.dt = 0.05;
  p.sigma << 0.3, 0.8;
  Controller mppi(DiffDriveModel{}, cost, p);
  mppi.EnableIntrospection(8, 8);
  mppi.Plan(Controller::State::Zero());

  quickviz::CvCanvas canvas(100);
  canvas.Resize(-0.5, 3.0, -1.0, 2.0);
  DrawMppiSnapshot2D(canvas, mppi.LastSnapshot());
  SUCCEED();
}
