/* 
 * cairo_draw.cpp
 * 
 * Created on: Oct 08, 2018 03:06
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "canvas/cairo_draw.hpp"

using namespace librav;

void CairoDraw::ShowCairo(CairoCallbackFunc_t context, std::string draw_name, int32_t win_width, int32_t win_height)
{
    CairoWindow win(context, draw_name, win_width, win_height);
    win.Run();
}