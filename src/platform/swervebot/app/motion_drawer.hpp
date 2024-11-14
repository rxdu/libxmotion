/*
 * @file motion_drawer.hpp
 * @date 11/14/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_MOTION_DRAWER_HPP
#define XMOTION_MOTION_DRAWER_HPP

#include "imview/widget/cairo_widget.hpp"

namespace xmotion {
class MotionDrawer : public quickviz::CairoWidget {
 public:
  MotionDrawer();
  ~MotionDrawer() = default;

 private:
  void DrawMotion(cairo_t* cr, float aspect_ratio);
};
}  // namespace xmotion

#endif  // XMOTION_MOTION_DRAWER_HPP