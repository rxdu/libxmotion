/*
 * @file sim_viewer.hpp
 * @brief Live 2D viewer for simulation runs.
 *
 * Attaches to a SimLoop through its observer callback: renders the world
 * (obstacles, goal), the executed trail, and any number of per-frame
 * overlays (e.g. the MPPI introspection drawer, estimator ellipses). The
 * headless loop is untouched — the viewer is a wrapper, not a fork.
 *
 * Frame pacing uses the frame period passed to the window (0 = wait for a
 * key each frame, i.e. single-step inspection).
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_VIZ_SIM_VIEWER_HPP
#define XMNAV_VIZ_SIM_VIEWER_HPP

#include <functional>
#include <string>
#include <vector>

#include "cvdraw/cvdraw.hpp"

namespace xmotion {

class SimViewer2D {
 public:
  struct Config {
    double x_min = -1.0, x_max = 5.0;
    double y_min = -2.0, y_max = 2.0;
    int pixels_per_unit = 120;
    int frame_period_ms = 20;  // 0 = single-step on keypress
    std::string window_name = "xmnav sim";
  };

  // draws onto the canvas each frame, before the trail
  using Overlay = std::function<void(quickviz::CvCanvas &)>;

  explicit SimViewer2D(const Config &config) : config_(config) {}

  void AddOverlay(Overlay overlay) { overlays_.push_back(std::move(overlay)); }

  void AddObstacle(const Eigen::Vector2d &center, double radius) {
    obstacles_.push_back({center, radius});
  }
  void SetGoal(const Eigen::Vector2d &goal) {
    goal_ = goal;
    has_goal_ = true;
  }

  // render one frame around the current 2D position (state rows x/y)
  void Frame(double x, double y) {
    quickviz::CvCanvas canvas(config_.pixels_per_unit);
    canvas.Resize(config_.x_min, config_.x_max, config_.y_min, config_.y_max);
    canvas.SetMode(quickviz::CvCanvas::DrawMode::Geometry);

    for (const auto &ob : obstacles_) {
      canvas.DrawCircle({ob.center(0), ob.center(1)}, ob.radius,
                        quickviz::CvColors::gray_color, 2);
    }
    if (has_goal_) {
      canvas.DrawPoint({goal_(0), goal_(1)}, 4,
                       quickviz::CvColors::green_color);
    }
    for (const auto &overlay : overlays_) {
      overlay(canvas);
    }

    trail_.push_back({x, y});
    for (std::size_t i = 1; i < trail_.size(); ++i) {
      canvas.DrawLine({trail_[i - 1].first, trail_[i - 1].second},
                      {trail_[i].first, trail_[i].second},
                      quickviz::CvColors::black_color, 2);
    }
    canvas.DrawPoint({x, y}, 3, quickviz::CvColors::blue_color);

    quickviz::CvIO::ShowImageFrame(canvas.GetPaintArea(), config_.window_name,
                                   config_.frame_period_ms);
  }

 private:
  struct Obstacle {
    Eigen::Vector2d center;
    double radius;
  };

  Config config_;
  std::vector<Overlay> overlays_;
  std::vector<Obstacle> obstacles_;
  Eigen::Vector2d goal_ = Eigen::Vector2d::Zero();
  bool has_goal_ = false;
  std::vector<std::pair<double, double>> trail_;
};

}  // namespace xmotion

#endif  // XMNAV_VIZ_SIM_VIEWER_HPP
