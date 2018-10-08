/* 
 * cairo_canvas.hpp
 * 
 * Created on: Mar 16, 2018 16:51
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef CAIRO_CANVAS_HPP
#define CAIRO_CANVAS_HPP

#include <memory>
#include <cstdint>
#include <functional>

#include <FL/x.H>
#include <FL/Fl.H>
#include <FL/Fl_Box.H>

#include <cairo.h>
#ifdef WIN32
#include <cairo-win32.h>
#elif defined(__APPLE__)
#include <cairo-quartz.h>
#else
#include <cairo-xlib.h>
#endif

namespace librav
{
using CairoCallbackFunc_t = std::function<void(cairo_t *cr, double, double, double, double)>;

enum class FileType
{
  PNG = 0,
  SVG,
  EPS,
  PDF
};

class CairoCanvas : public Fl_Box
{
public:
  CairoCanvas(int32_t x, int32_t y, int32_t w, int32_t h);

  friend class CairoWindow;

  // virtual void CairoDraw(cairo_t *cr, double, double, double, double);
  void SaveToFile(std::string filename, FileType type, int32_t wpts = -1, int32_t hpts = -1);
  inline void SetCairoCallback(CairoCallbackFunc_t func) { cairo_callback_ = func; }

private:
  // callback function for cario plotting
  std::function<void(cairo_t *cr, double, double, double, double)> cairo_callback_;

  // draw function for box
  void draw(void);

  // this function creates a surface for current platform (win/linux/mac)
  cairo_surface_t *cairo_gui_surface_create(int wo, int ho);
};
} // namespace librav

#endif /* CAIRO_CANVAS_HPP */
