/* 
 * cairo_draw.hpp
 * 
 * Created on: Oct 08, 2018 03:00
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef CAIRO_DRAW_HPP
#define CAIRO_DRAW_HPP

#include <cstdint>

#include "canvas/details/cairo_window.hpp"

namespace librav
{
namespace CairoDraw
{
  void ShowCairo(CairoCallbackFunc_t context, std::string draw_name = "cairo", int32_t win_width = 700, int32_t win_height = 600);
}
} // namespace librav

#endif /* CAIRO_DRAW_HPP */
