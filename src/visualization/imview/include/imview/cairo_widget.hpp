/*
 * cairo_widget.hpp
 *
 * Created on: Mar 05, 2021 10:42
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef CAIRO_WIDGET_HPP
#define CAIRO_WIDGET_HPP

#include <cmath>
#include <string>
#include <cstdint>
#include <memory>
#include <functional>
#include <unordered_map>

#include "imgui.h"

#include "imview/details/cairo_context.hpp"

namespace xmotion {
namespace swviz {
class CairoWidget {
 public:
  CairoWidget(uint32_t width, uint32_t height,
              bool normalize_coordinate = false);
  ~CairoWidget();

  // load image texture (before entering rendering loop)
  void LoadImage(std::string png_file);

  // resize/fill cairo surface
  void Resize(uint32_t width, uint32_t height);
  void Fill(ImVec4 color = {1, 1, 1, 0.6});
  void Clear();

  float GetAspectRatio() const;

  // draw vector graphics with user function
  using CairoDrawFunc = std::function<void(cairo_t*)>;
  void Draw(CairoDrawFunc DrawFunc);

  // draw from png image (avoid if possible, slow)
  enum class ScaleMode { MANUAL, AUTO_STRETCH, AUTO_KEEP_ASPECT_RATIO };
  void Draw(std::string png_file, double pos_x, double pos_y,
            double angle = 0.0,
            ScaleMode scale_mode = ScaleMode::AUTO_KEEP_ASPECT_RATIO,
            double scale_x = 1.0, double scale_y = 1.0);

  // draw text to cairo surface
  void DrawText(std::string text, double pos_x, double pos_y,
                double angle = 0.0, ImVec4 color = {0, 0, 0, 1},
                double size = 14.0, double line_width = 3.0,
                const char* font = "Sans");

  // render cairo content to OpenGL context to display with ImGUI
  void Render(const ImVec2& uv0 = ImVec2(0, 0),
              const ImVec2& uv1 = ImVec2(1, 1),
              const ImVec4& tint_col = ImVec4(1, 1, 1, 1),
              const ImVec4& border_col = ImVec4(0, 0, 0, 0));

 private:
  std::unique_ptr<CairoContext> ctx_;
  std::unordered_map<std::string, cairo_surface_t*> images_;

  cairo_surface_t* GetImageSurface(std::string png_file);
};
}  // namespace swviz
}  // namespace xmotion

#endif /* CAIRO_WIDGET_HPP */
