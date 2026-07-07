/*
 * @file mppi_draw_cairo.hpp
 * @brief Cairo (vector) rendering of MPPI introspection snapshots.
 *
 * Counterpart of mppi_draw.hpp for the quickviz canvas module: candidate
 * rollouts are drawn as anti-aliased polylines whose *alpha* encodes the
 * softmax weight, so overlapping candidates composite into a density fan —
 * something the raster CvCanvas drawer cannot do (it encodes weight as
 * intensity and later strokes overwrite earlier ones).
 *
 * Coordinates: all drawers take a CairoWorldFrame that maps world meters
 * into the normalized coordinates of a CairoWidget constructed with
 * normalize_coordinate = true (x in [0, aspect_ratio], y in [0, 1],
 * y-down). The mapping preserves aspect (meters stay square) and centers
 * the world rect in the panel.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_VIZ_MPPI_DRAW_CAIRO_HPP
#define XMNAV_VIZ_MPPI_DRAW_CAIRO_HPP

#include <algorithm>
#include <cmath>

#include "canvas/details/cairo_draw.hpp"

#include "xmnav/mppi/mppi.hpp"

namespace xmotion {

// World-to-canvas mapping for a normalized-coordinate CairoWidget.
struct CairoWorldFrame {
  double x_min = 0.0;
  double x_max = 1.0;
  double y_min = 0.0;
  double y_max = 1.0;

  // meters -> canvas units (canvas unit = panel height); aspect-preserving
  double Scale(float aspect_ratio) const {
    const double sx = aspect_ratio / (x_max - x_min);
    const double sy = 1.0 / (y_max - y_min);
    return std::min(sx, sy);
  }

  ImVec2 ToCanvas(double wx, double wy, float aspect_ratio) const {
    const double s = Scale(aspect_ratio);
    const double ox = 0.5 * (aspect_ratio - s * (x_max - x_min));
    const double oy = 0.5 * (1.0 - s * (y_max - y_min));
    return ImVec2(static_cast<float>(ox + s * (wx - x_min)),
                  static_cast<float>(1.0 - oy - s * (wy - y_min)));
  }
};

namespace viz_style {
// cairo colors (rgba in [0,1]); the CvScalar palette in style.hpp is BGR-byte
inline const ImVec4 kCairoBackground = {1.0f, 1.0f, 1.0f, 1.0f};
inline const ImVec4 kCairoCandidateFan = {0.86f, 0.20f, 0.15f, 1.0f};
inline const ImVec4 kCairoChosenTrajectory = {0.12f, 0.35f, 0.90f, 1.0f};
inline const ImVec4 kCairoExecutedTrail = {0.15f, 0.15f, 0.15f, 0.9f};
inline const ImVec4 kCairoGoal = {0.10f, 0.65f, 0.25f, 0.9f};
inline const ImVec4 kCairoObstacle = {0.55f, 0.55f, 0.55f, 0.8f};
inline const ImVec4 kCairoRobot = {0.10f, 0.10f, 0.10f, 1.0f};
}  // namespace viz_style

namespace viz_cairo_detail {
// stroke rows [0, rows) of a state matrix as one cairo path
template <typename StateMatrix>
inline void StrokeStates(cairo_t *cr, const CairoWorldFrame &frame,
                         float aspect_ratio, const StateMatrix &states,
                         int x_col, int y_col, double width,
                         const ImVec4 &color, double alpha) {
  if (states.rows() < 2) return;
  cairo_save(cr);
  cairo_set_source_rgba(cr, color.x, color.y, color.z, alpha);
  cairo_set_line_width(cr, width);
  cairo_set_line_join(cr, CAIRO_LINE_JOIN_ROUND);
  const ImVec2 p0 =
      frame.ToCanvas(states(0, x_col), states(0, y_col), aspect_ratio);
  cairo_move_to(cr, p0.x, p0.y);
  for (Eigen::Index t = 1; t < states.rows(); ++t) {
    const ImVec2 p =
        frame.ToCanvas(states(t, x_col), states(t, y_col), aspect_ratio);
    cairo_line_to(cr, p.x, p.y);
  }
  cairo_stroke(cr);
  cairo_restore(cr);
}
}  // namespace viz_cairo_detail

// Candidate fan (alpha ~ softmax weight, normalized to the strongest
// candidate) with the updated nominal trajectory on top. Any two state
// columns can serve as the drawing plane, as in DrawMppiSnapshot2D.
template <int StateDim, int ControlDim>
inline void DrawMppiSnapshotCairo(cairo_t *cr, float aspect_ratio,
                                  const CairoWorldFrame &frame,
                                  const MppiSnapshot<StateDim, ControlDim> &snap,
                                  int x_col = 0, int y_col = 1) {
  double w_max = 1e-12;
  for (const auto &c : snap.candidates) w_max = std::max(w_max, c.weight);

  for (const auto &c : snap.candidates) {
    // floor keeps discarded candidates faintly visible (sampling coverage);
    // influential ones composite into an opaque core
    const double alpha = 0.04 + 0.56 * (c.weight / w_max);
    viz_cairo_detail::StrokeStates(cr, frame, aspect_ratio, c.states, x_col,
                                   y_col, 0.0018,
                                   viz_style::kCairoCandidateFan, alpha);
  }
  viz_cairo_detail::StrokeStates(cr, frame, aspect_ratio, snap.nominal_states,
                                 x_col, y_col, 0.004,
                                 viz_style::kCairoChosenTrajectory, 1.0);
}

inline void DrawDiscCairo(cairo_t *cr, float aspect_ratio,
                          const CairoWorldFrame &frame,
                          const Eigen::Vector2d &center, double radius_m,
                          const ImVec4 &color, bool fill = true) {
  const ImVec2 c = frame.ToCanvas(center.x(), center.y(), aspect_ratio);
  const double r = radius_m * frame.Scale(aspect_ratio);
  cairo_save(cr);
  cairo_set_source_rgba(cr, color.x, color.y, color.z, color.w);
  cairo_arc(cr, c.x, c.y, r, 0.0, 2.0 * M_PI);
  if (fill) {
    cairo_fill(cr);
  } else {
    cairo_set_line_width(cr, 0.003);
    cairo_stroke(cr);
  }
  cairo_restore(cr);
}

// body disc + heading tick
inline void DrawRobotCairo(cairo_t *cr, float aspect_ratio,
                           const CairoWorldFrame &frame, double x, double y,
                           double theta, double radius_m,
                           const ImVec4 &color = viz_style::kCairoRobot) {
  DrawDiscCairo(cr, aspect_ratio, frame, {x, y}, radius_m, color, false);
  const ImVec2 c = frame.ToCanvas(x, y, aspect_ratio);
  const ImVec2 h = frame.ToCanvas(x + radius_m * std::cos(theta),
                                  y + radius_m * std::sin(theta),
                                  aspect_ratio);
  cairo_save(cr);
  cairo_set_source_rgba(cr, color.x, color.y, color.z, color.w);
  cairo_set_line_width(cr, 0.003);
  cairo_move_to(cr, c.x, c.y);
  cairo_line_to(cr, h.x, h.y);
  cairo_stroke(cr);
  cairo_restore(cr);
}

// executed path as a polyline; Trail is any container of Eigen::Vector2d
template <typename Trail>
inline void DrawTrailCairo(cairo_t *cr, float aspect_ratio,
                           const CairoWorldFrame &frame, const Trail &trail,
                           const ImVec4 &color = viz_style::kCairoExecutedTrail) {
  if (trail.size() < 2) return;
  cairo_save(cr);
  cairo_set_source_rgba(cr, color.x, color.y, color.z, color.w);
  cairo_set_line_width(cr, 0.002);
  auto it = trail.begin();
  ImVec2 p = frame.ToCanvas((*it).x(), (*it).y(), aspect_ratio);
  cairo_move_to(cr, p.x, p.y);
  for (++it; it != trail.end(); ++it) {
    p = frame.ToCanvas((*it).x(), (*it).y(), aspect_ratio);
    cairo_line_to(cr, p.x, p.y);
  }
  cairo_stroke(cr);
  cairo_restore(cr);
}

}  // namespace xmotion

#endif  // XMNAV_VIZ_MPPI_DRAW_CAIRO_HPP
