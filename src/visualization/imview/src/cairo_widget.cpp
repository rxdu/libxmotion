/*
 * cairo_widget.cpp
 *
 * Created on: Mar 05, 2021 10:46
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include "imview/cairo_widget.hpp"

#include <cmath>
#include <iostream>

#include <fontconfig/fontconfig.h>

namespace xmotion {
namespace swviz {
CairoWidget::CairoWidget(uint32_t width, uint32_t height,
                         bool normalize_coordinate)
    : ctx_(new CairoContext(width, height, normalize_coordinate)) {}

CairoWidget::~CairoWidget() {
  // font-related memory cleanup
  // Reference:
  // [1]
  // https://stackoverflow.com/questions/51174295/cairo-show-text-memory-leak
  // [2] https://gitlab.freedesktop.org/cairo/cairo/-/issues/393
  cairo_debug_reset_static_data();
  FcFini();

  // cleanup image surfaces
  for (auto& img : images_) {
    cairo_surface_destroy(img.second);
  }
}

void CairoWidget::Fill(ImVec4 color) {
  auto cr = ctx_->GetCairoObject();
  cairo_set_source_rgba(cr, color.x, color.y, color.z, color.w);
  cairo_paint(cr);
}

float CairoWidget::GetAspectRatio() const { return ctx_->GetAspectRatio(); }

void CairoWidget::Clear() {
  // Reference:
  // [1] https://www.cairographics.org/FAQ/#clear_a_surface
  auto cr = ctx_->GetCairoObject();
  cairo_set_operator(cr, CAIRO_OPERATOR_SOURCE);
  cairo_paint(cr);
}

void CairoWidget::Draw(CairoDrawFunc DrawFunc) {
  assert(ctx_ != nullptr && ctx_->GetCairoObject() != NULL);

  // do actual paint with cairo
  DrawFunc(ctx_->GetCairoObject());
}

void CairoWidget::LoadImage(std::string png_file) {
  assert(images_.find(png_file) == images_.end());
  auto surface = cairo_image_surface_create_from_png(png_file.c_str());
  images_[png_file] = surface;
}

cairo_surface_t* CairoWidget::GetImageSurface(std::string png_file) {
  assert(images_.find(png_file) != images_.end());
  return images_[png_file];
}

void CairoWidget::Resize(uint32_t width, uint32_t height) {
  ctx_->Resize(width, height);
}

void CairoWidget::DrawText(std::string text, double pos_x, double pos_y,
                           double angle, ImVec4 color, double size,
                           double line_width, const char* font) {
  auto cr = ctx_->GetCairoObject();

  cairo_save(cr);

  cairo_select_font_face(cr, font, CAIRO_FONT_SLANT_NORMAL,
                         CAIRO_FONT_WEIGHT_NORMAL);
  cairo_set_font_size(cr, size);

  //   cairo_text_extents_t extents;
  //   cairo_text_extents(cr, text.c_str(), &extents);

  cairo_set_source_rgba(cr, color.x, color.y, color.z, color.w);
  cairo_set_line_width(cr, line_width);

  cairo_move_to(cr, pos_x, pos_y);
  cairo_show_text(cr, text.c_str());

  cairo_restore(cr);
}

void CairoWidget::Draw(std::string png_file, double pos_x, double pos_y,
                       double angle, ScaleMode scale_mode, double scale_x,
                       double scale_y) {
  auto cr = ctx_->GetCairoObject();

  auto surface = GetImageSurface(png_file);
  double sx = 1.0, sy = 1.0;
  double pos_offset_x = 0, pos_offset_y = 0;

  if (scale_mode != ScaleMode::MANUAL) {
    auto width = cairo_image_surface_get_width(surface);
    auto height = cairo_image_surface_get_height(surface);

    auto region = ImGui::GetContentRegionAvail();
    sx = region.x / width;
    sy = region.y / height;

    if (scale_mode == ScaleMode::AUTO_KEEP_ASPECT_RATIO) {
      double ratio = width / height;
      double dx = region.y * ratio, dy = 0;
      if (dx > region.x) {
        // height is too high, y offset needed
        dx = region.x;
        dy = dx / ratio;
        pos_offset_y = (region.y - dy) / 2.0f;
      } else {
        // width is too wide, x offset needed
        dy = region.y;
        dx = dy * ratio;
        pos_offset_x = (region.x - dx) / 2.0f;
      }
      sx = dx / width;
      sy = dy / height;
    }
  } else {
    sx = scale_x;
    sy = scale_y;
  }

  cairo_save(cr);

  cairo_translate(cr, pos_x + pos_offset_x, pos_y + pos_offset_y);
  cairo_rotate(cr, angle);

  cairo_scale(cr, sx, sy);
  cairo_set_source_surface(cr, surface, 0, 0);
  cairo_paint(cr);

  cairo_restore(cr);
}

void CairoWidget::Render(const ImVec2& uv0, const ImVec2& uv1,
                         const ImVec4& tint_col, const ImVec4& border_col) {
  GLuint image = ctx_->RenderToGlTexture();
  ImGui::Image((void*)(intptr_t)image, ImGui::GetContentRegionAvail(), uv0, uv1,
               tint_col, border_col);
}
}  // namespace swviz
}  // namespace xmotion