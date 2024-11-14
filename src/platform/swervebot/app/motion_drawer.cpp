/*
 * @file motion_drawer.cpp
 * @date 11/14/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "motion_drawer.hpp"

#include <iostream>

namespace xmotion {
using namespace quickviz;

namespace {
void DrawRobotFootprint(cairo_t* cr, float aspect_ratio) {
  float pos_x = 0.5 * aspect_ratio;
  float pos_y = 0.5;

  // draw robot footprint
  DrawPoint(cr, {pos_x, pos_y}, 0.01, {0, 0.4, 1.0, 0.6});
  DrawRectangle(cr, {pos_x - 0.2f, pos_y - 0.25f},
                {pos_x + 0.2f, pos_y + 0.25f}, 0.002, {0.2, 0.2, 0.2, 0.6});
}

void DrawSpeed(cairo_t* cr, float aspect_ratio, float x, float y, float linear,
               float angle, float radius = 0.1) {
  float pos_x = x;
  float pos_y = y;

  // draw speed
  DrawCircle(cr, {pos_x, pos_y}, radius, 0.001, {0.2, 0.2, 0.2, 0.6});
}
}  // namespace

MotionDrawer::MotionDrawer() : CairoWidget("cairo", true) {
  this->SetNoTitleBar(true);
  this->SetAutoLayout(true);
  this->SetNoResize(true);

  AttachDrawFunction(std::bind(&MotionDrawer::DrawMotion, this,
                               std::placeholders::_1, std::placeholders::_2));
}

void MotionDrawer::DrawMotion(cairo_t* cr, float aspect_ratio) {
  float cx = 0.5 * aspect_ratio;
  float cy = 0.5;

  DrawRobotFootprint(cr, aspect_ratio);

  DrawSpeed(cr, aspect_ratio, cx, cy, 0.1, 0.2, 0.12);

  DrawSpeed(cr, aspect_ratio, cx - 0.2f, cy - 0.25f, 0.1, 0.2);
  DrawSpeed(cr, aspect_ratio, cx + 0.2f, cy - 0.25f, 0.1, 0.2);
  DrawSpeed(cr, aspect_ratio, cx - 0.2f, cy + 0.25f, 0.1, 0.2);
  DrawSpeed(cr, aspect_ratio, cx + 0.2f, cy + 0.25f, 0.1, 0.2);
}
}  // namespace xmotion