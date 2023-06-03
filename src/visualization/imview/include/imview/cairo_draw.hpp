/*
 * cairo_draw.hpp
 *
 * Created on: Jul 29, 2021 15:23
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef CAIRO_DRAW_HPP
#define CAIRO_DRAW_HPP

#include <cairo.h>

#include "imgui.h"

namespace xmotion {
namespace swviz {
enum ColorName {
  BLACK = 0,
  WHITE,
  BLUE,
  LIME,
  RED,
  YELLOW,
  CYAN,
  MAGENTA,
  SILVER,
  GRAY,
  MAROON,
  OLIVE,
  GREEN,
  PURPLE,
  COLOR_LAST
};

const ImVec4 colors[COLOR_LAST] = {
    {0, 0, 0, 1},       {1, 1, 1, 1},    {0, 0, 1, 1},
    {0, 1, 0, 1},       {1, 0, 0, 1},    {1, 1, 0, 1},
    {0, 1, 1, 1},       {1, 0, 1, 1},    {0.7529, 0.7529, 0.7529, 1},
    {0.5, 0.5, 0.5, 1}, {0, 0, 0.5, 1},  {0.5, 0.5, 0, 1},
    {0, 0.5, 0, 1},     {0.5, 0, 0.5, 1}};

void DrawPoint(cairo_t *cr, ImVec2 pos, double size = 2,
               ImVec4 color = colors[BLACK]);

void DrawLine(cairo_t *cr, ImVec2 pos1, ImVec2 pos2, double thickness = 2,
              ImVec4 color = colors[BLACK]);

void DrawArc(cairo_t *cr, ImVec2 center, float radius, float start_angle,
             float end_angle, double thickness = 2,
             ImVec4 color = colors[BLACK]);

void DrawArcSector(cairo_t *cr, ImVec2 center, float radius, float start_angle,
                   float end_angle, double thickness = 2,
                   ImVec4 color = colors[BLACK], bool fill = false);

void DrawCircle(cairo_t *cr, ImVec2 center, float radius, double thickness = 2,
                ImVec4 color = colors[BLACK]);

void DrawRing(cairo_t *cr, ImVec2 center, float inner_radius,
              float outer_radius, float start_angle, float end_angle,
              double thickness = 2, ImVec4 color = colors[BLACK],
              bool fill = false);

void DrawRectangle(cairo_t *cr, ImVec2 pos1, ImVec2 pos2, double thickness = 2,
                   ImVec4 color = colors[BLACK], bool fill = false);
}  // namespace swviz
}  // namespace xmotion

#endif /* CAIRO_DRAW_HPP */
