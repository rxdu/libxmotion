/*
 * @file sim_viewer.hpp
 * @brief Live 2D viewer for simulation runs.
 *
 * Attaches to a SimLoop through its observer callback: renders the world
 * (obstacles, goal), the executed trail, and any number of per-frame
 * overlays (e.g. the MPPI introspection drawer, estimator ellipses). The
 * headless loop is untouched — the viewer is a wrapper, not a fork.
 *
 * Presentation modes (combinable):
 *  - window: frame pacing via frame_period_ms (0 = single-step per key)
 *  - recording: numbered PNGs into record_dir — works fully headless, so
 *    demo runs can produce CI artifacts / videos (ffmpeg over the frames)
 *
 * Module rule: Draw/To functions in this module never present; this
 * class is where presentation lives.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_VIZ_SIM_VIEWER_HPP
#define XMNAV_VIZ_SIM_VIEWER_HPP

#include <cstdio>
#include <deque>
#include <functional>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "image/image.hpp"

#include "xmnav/viz/style.hpp"

namespace xmotion {

class SimViewer2D {
 public:
  struct Config {
    double x_min = -1.0, x_max = 5.0;
    double y_min = -2.0, y_max = 2.0;
    int pixels_per_unit = 120;
    int frame_period_ms = 20;  // 0 = single-step on keypress
    bool show_window = true;
    std::string window_name = "xmnav sim";
    // when non-empty, every frame is written as <record_dir>/frame_N.png
    std::string record_dir;
    // executed-trail history bound (decimated by 2 when exceeded)
    std::size_t max_trail_points = 4096;
  };

  using Overlay = std::function<void(quickviz::CvCanvas &)>;

  explicit SimViewer2D(const Config &config)
      : config_(config), canvas_(config.pixels_per_unit) {
    canvas_.Resize(config_.x_min, config_.x_max, config_.y_min, config_.y_max);
    canvas_.SetMode(quickviz::CvCanvas::DrawMode::Geometry);
  }

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
    canvas_.Clear();

    for (const auto &ob : obstacles_) {
      canvas_.DrawCircle({ob.center(0), ob.center(1)}, ob.radius,
                         viz_style::kObstacle, 2);
    }
    if (has_goal_) {
      canvas_.DrawPoint({goal_(0), goal_(1)}, 4, viz_style::kGoal);
    }
    for (const auto &overlay : overlays_) {
      overlay(canvas_);
    }

    trail_.push_back({x, y});
    if (trail_.size() > config_.max_trail_points) {
      Decimate();
    }
    for (std::size_t i = 1; i < trail_.size(); ++i) {
      canvas_.DrawLine({trail_[i - 1].first, trail_[i - 1].second},
                       {trail_[i].first, trail_[i].second},
                       viz_style::kExecutedTrail, 2);
    }
    canvas_.DrawPoint({x, y}, 3, viz_style::kChosenTrajectory);

    Present();
  }

 private:
  void Decimate() {
    std::deque<std::pair<double, double>> kept;
    for (std::size_t i = 0; i < trail_.size(); i += 2) {
      kept.push_back(trail_[i]);
    }
    trail_.swap(kept);
  }

  void Present() {
    if (!config_.record_dir.empty()) {
      char name[512];
      std::snprintf(name, sizeof(name), "%s/frame_%06d.png",
                    config_.record_dir.c_str(), frame_index_);
      cv::imwrite(name, canvas_.GetPaintArea());
    }
    if (config_.show_window) {
      quickviz::CvIO::ShowImageFrame(canvas_.GetPaintArea(),
                                     config_.window_name,
                                     config_.frame_period_ms);
    }
    ++frame_index_;
  }

  struct Obstacle {
    Eigen::Vector2d center;
    double radius;
  };

  Config config_;
  quickviz::CvCanvas canvas_;
  std::vector<Overlay> overlays_;
  std::vector<Obstacle> obstacles_;
  Eigen::Vector2d goal_ = Eigen::Vector2d::Zero();
  bool has_goal_ = false;
  std::deque<std::pair<double, double>> trail_;
  int frame_index_ = 0;
};

}  // namespace xmotion

#endif  // XMNAV_VIZ_SIM_VIEWER_HPP
