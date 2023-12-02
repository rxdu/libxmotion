/*
 * cairo_draw.cpp
 *
 * Created on: Jul 29, 2021 15:32
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include "imview/cairo_draw.hpp"

#include <cmath>

namespace xmotion {
namespace swviz {
void DrawPoint(cairo_t *cr, ImVec2 pos, double size, ImVec4 color) {
  cairo_set_source_rgba(cr, color.x, color.y, color.z, color.w);
  cairo_arc(cr, pos.x, pos.y, size, 0, 2 * M_PI);
  cairo_fill(cr);
}

void DrawLine(cairo_t *cr, ImVec2 pos1, ImVec2 pos2, double thickness,
              ImVec4 color) {
  cairo_set_source_rgba(cr, color.x, color.y, color.z, color.w);
  cairo_set_line_width(cr, thickness);

  cairo_move_to(cr, pos1.x, pos1.y);
  cairo_line_to(cr, pos2.x, pos2.y);
  cairo_stroke(cr);
}

void DrawArc(cairo_t *cr, ImVec2 center, float radius, float start_angle,
             float end_angle, double thickness, ImVec4 color) {
  cairo_set_source_rgba(cr, color.x, color.y, color.z, color.w);
  cairo_set_line_width(cr, thickness);

  cairo_new_sub_path(cr);
  cairo_arc(cr, center.x, center.y, radius, start_angle, end_angle);
  cairo_stroke(cr);
}

void DrawArcSector(cairo_t *cr, ImVec2 center, float radius, float start_angle,
                   float end_angle, double thickness, ImVec4 color, bool fill) {
  if (!fill) {
    cairo_set_line_cap(cr, CAIRO_LINE_CAP_ROUND);

    cairo_set_source_rgba(cr, color.x, color.y, color.z, color.w);
    cairo_set_line_width(cr, thickness);

    cairo_new_sub_path(cr);
    cairo_arc(cr, center.x, center.y, radius, start_angle, end_angle);
    cairo_stroke(cr);

    ImVec2 pos1{center.x + radius * std::cos(start_angle),
                center.y + radius * std::sin(start_angle)};
    ImVec2 pos2{center.x + radius * std::cos(end_angle),
                center.y + radius * std::sin(end_angle)};
    DrawLine(cr, center, pos1, thickness, color);
    DrawLine(cr, center, pos2, thickness, color);

    cairo_set_line_cap(cr, CAIRO_LINE_CAP_BUTT);
  } else {
    ImVec2 pos1{center.x + radius * std::cos(start_angle),
                center.y + radius * std::sin(start_angle)};
    ImVec2 pos2{center.x + radius * std::cos(end_angle),
                center.y + radius * std::sin(end_angle)};

    cairo_set_source_rgba(cr, color.x, color.y, color.z, color.w);

    cairo_move_to(cr, center.x, center.y);
    cairo_line_to(cr, pos1.x, pos1.y);
    cairo_new_sub_path(cr);
    cairo_arc(cr, center.x, center.y, radius, start_angle, end_angle);
    cairo_line_to(cr, center.x, center.y);
    cairo_fill(cr);
  }
}

void DrawCircle(cairo_t *cr, ImVec2 center, float radius, double thickness,
                ImVec4 color) {
  cairo_set_source_rgba(cr, color.x, color.y, color.z, color.w);
  cairo_set_line_width(cr, thickness);

  cairo_new_sub_path(cr);
  cairo_arc(cr, center.x, center.y, radius, 0, 2 * M_PI);
  cairo_stroke(cr);
}

void DrawRing(cairo_t *cr, ImVec2 center, float inner_radius,
              float outer_radius, float start_angle, float end_angle,
              double thickness, ImVec4 color, bool fill) {
  if (!fill) {
    cairo_set_line_cap(cr, CAIRO_LINE_CAP_ROUND);

    DrawArc(cr, center, inner_radius, start_angle, end_angle, thickness, color);
    DrawArc(cr, center, outer_radius, start_angle, end_angle, thickness, color);

    ImVec2 pos1{center.x + inner_radius * std::cos(start_angle),
                center.y + inner_radius * std::sin(start_angle)};
    ImVec2 pos2{center.x + outer_radius * std::cos(start_angle),
                center.y + outer_radius * std::sin(start_angle)};
    ImVec2 pos3{center.x + inner_radius * std::cos(end_angle),
                center.y + inner_radius * std::sin(end_angle)};
    ImVec2 pos4{center.x + outer_radius * std::cos(end_angle),
                center.y + outer_radius * std::sin(end_angle)};
    DrawLine(cr, pos1, pos2, thickness, color);
    DrawLine(cr, pos3, pos4, thickness, color);

    cairo_set_line_cap(cr, CAIRO_LINE_CAP_BUTT);
  } else {
    DrawArc(cr, center, (inner_radius + outer_radius) / 2.0f, start_angle,
            end_angle, outer_radius - inner_radius, color);
  }
}

void DrawRectangle(cairo_t *cr, ImVec2 pos1, ImVec2 pos2, double thickness,
                   ImVec4 color, bool fill) {
  if (!fill) {
    cairo_set_line_cap(cr, CAIRO_LINE_CAP_ROUND);

    ImVec2 pt1 = pos1;
    ImVec2 pt2{pos1.x, pos2.y};
    ImVec2 pt3 = pos2;
    ImVec2 pt4{pos2.x, pos1.y};

    DrawLine(cr, pt1, pt2, thickness, color);
    DrawLine(cr, pt2, pt3, thickness, color);
    DrawLine(cr, pt3, pt4, thickness, color);
    DrawLine(cr, pt4, pt1, thickness, color);

    cairo_set_line_cap(cr, CAIRO_LINE_CAP_BUTT);
  } else {
    ImVec2 start{(pos1.x + pos2.x) / 2.0f, pos1.y};
    ImVec2 end{(pos1.x + pos2.x) / 2.0f, pos2.y};

    DrawLine(cr, start, end, std::abs(pos1.x - pos2.x), color);
  }
}
}  // namespace swviz
}  // namespace xmotion