/* 
 * cairo_utils.hpp
 * 
 * Created on: Mar 16, 2018 16:51
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef CAIRO_UTILS_HPP
#define CAIRO_UTILS_HPP

#include <memory>
#include <cstdint>
#include <functional>

#include <FL/x.H>
#include <FL/Fl.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Group.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Double_Window.H>

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
enum class FileType
{
  PNG = 0,
  SVG,
  EPS,
  PDF
};

using CallBackFunc_t = std::function<void(cairo_t *cr, double, double, double, double)>;

class CairoCanvas : public Fl_Box
{
public:
  CairoCanvas(int x, int y, int w, int h);

  friend class CairoWindow;

public:
  // virtual void CairoDraw(cairo_t *cr, double, double, double, double);
  void SaveToFile(std::string filename, FileType type, int32_t wpts = -1, int32_t hpts = -1);
  inline void SetCairoCallback(CallBackFunc_t func) { cairo_callback_ = func; }

private:
  // callback function for cario plotting
  std::function<void(cairo_t *cr, double, double, double, double)> cairo_callback_;

  // draw function for box
  void draw(void);
  // this function creates a surface for current platform (win/linux/mac)
  cairo_surface_t *cairo_gui_surface_create(int wo, int ho);
};

class CairoWindow
{
public:
  CairoWindow();
  CairoWindow(CallBackFunc_t cairo_cb);
  ~CairoWindow() = default;

  const int w = 700;
  const int h = 600;
  const int sp = 5;
  const int bw = 75;
  const int bh = 25;

  const int wpts = 175; // width in points
  const int hpts = 175; // height in points
  const int wpix = 175; // width in pixels
  const int hpix = 175; // height in pixels

  void Run();
  void SetCairoCallback(CallBackFunc_t func);

private:
  Fl_Group *button_group;
  Fl_Button *quit;
  Fl_Button *png;
  Fl_Button *svg;
  Fl_Button *eps;
  Fl_Button *pdf;
  Fl_Box *spacer;
  CairoCanvas *canvas;
  Fl_Double_Window *win;

  void CallbackBtnQuit(Fl_Widget *);
  void CallbackBtnPNG(Fl_Widget *);
  void CallbackBtnSVG(Fl_Widget *);
  void CallbackBtnEPS(Fl_Widget *);
  void CallbackBtnPDF(Fl_Widget *);

  void SetupWindowElements();

  // Reference: http://www.fltk.org/articles.php?L379+I0+TFAQ+P1+Q
  static void StaticCallback_BtnQuit(Fl_Widget *w, void *data)
  {
    CairoWindow *win_ptr = (CairoWindow *)data;
    win_ptr->CallbackBtnQuit(w);
  }

  static void StaticCallback_BtnPNG(Fl_Widget *w, void *data)
  {
    CairoWindow *win_ptr = (CairoWindow *)data;
    win_ptr->CallbackBtnPNG(w);
  }

  static void StaticCallback_BtnSVG(Fl_Widget *w, void *data)
  {
    CairoWindow *win_ptr = (CairoWindow *)data;
    win_ptr->CallbackBtnSVG(w);
  }

  static void StaticCallback_BtnEPS(Fl_Widget *w, void *data)
  {
    CairoWindow *win_ptr = (CairoWindow *)data;
    win_ptr->CallbackBtnEPS(w);
  }

  static void StaticCallback_BtnPDF(Fl_Widget *w, void *data)
  {
    CairoWindow *win_ptr = (CairoWindow *)data;
    win_ptr->CallbackBtnPDF(w);
  }
};
}

#endif /* CAIRO_UTILS_HPP */
