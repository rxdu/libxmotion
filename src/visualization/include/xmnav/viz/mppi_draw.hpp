/*
 * @file mppi_draw.hpp
 * @brief Live 2D rendering of MPPI introspection snapshots.
 *
 * Draws the candidate rollouts as polylines whose intensity encodes their
 * softmax weight (dim = discarded, bright = influential), and the updated
 * nominal trajectory on top. Any two state columns can serve as the plane
 * (x/y for planar robots by default), so the same drawer works for the
 * diff-drive, Ackermann, and SRB ground-plane projections.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_VIZ_MPPI_DRAW_HPP
#define XMNAV_VIZ_MPPI_DRAW_HPP

#include <algorithm>

#include "cvdraw/cvdraw.hpp"

#include "xmnav/mppi/mppi.hpp"
#include "xmnav/viz/style.hpp"

namespace xmotion {

template <int StateDim, int ControlDim>
void DrawMppiSnapshot2D(quickviz::CvCanvas &canvas,
                        const MppiSnapshot<StateDim, ControlDim> &snap,
                        int x_col = 0, int y_col = 1) {
  double w_max = 1e-12;
  for (const auto &c : snap.candidates) w_max = std::max(w_max, c.weight);

  for (const auto &c : snap.candidates) {
    // weight -> intensity: influential candidates draw bright
    const cv::Scalar color = viz_style::WeightColor(c.weight / w_max);
    for (Eigen::Index t = 0; t + 1 < c.states.rows(); ++t) {
      canvas.DrawLine({c.states(t, x_col), c.states(t, y_col)},
                      {c.states(t + 1, x_col), c.states(t + 1, y_col)}, color,
                      1);
    }
  }

  // the chosen (updated nominal) trajectory on top
  for (Eigen::Index t = 0; t + 1 < snap.nominal_states.rows(); ++t) {
    canvas.DrawLine(
        {snap.nominal_states(t, x_col), snap.nominal_states(t, y_col)},
        {snap.nominal_states(t + 1, x_col), snap.nominal_states(t + 1, y_col)},
        viz_style::kChosenTrajectory, 2);
  }
}

}  // namespace xmotion

#endif  // XMNAV_VIZ_MPPI_DRAW_HPP
