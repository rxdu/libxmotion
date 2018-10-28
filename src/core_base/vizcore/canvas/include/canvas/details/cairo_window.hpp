/* 
 * cairo_window.hpp
 * 
 * Created on: Oct 08, 2018 02:52
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef CAIRO_WINDOW_HPP
#define CAIRO_WINDOW_HPP

#include <memory>
#include <cstdint>
#include <functional>

#include <FL/x.H>
#include <FL/Fl.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Group.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Double_Window.H>

#include "canvas/details/cairo_canvas.hpp"

namespace librav
{
class CairoWindow
{
public:
  CairoWindow() = delete;
  CairoWindow(CairoCallbackFunc_t cairo_cb, std::string winname = "cairo", int32_t width = 700, int32_t height = 600);
  ~CairoWindow() = default;

  int32_t win_width_;
  int32_t win_height_;

  const int32_t vertical_spacer_ = 5;
  const int32_t btn_width_ = 75;
  const int32_t btn_height_ = 25;

  // const int wpts = 175; // width in points
  // const int hpts = 175; // height in points
  // const int wpix = 175; // width in pixels
  // const int hpix = 175; // height in pixels

  void Run();
  void SetCairoCallback(CairoCallbackFunc_t func);

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

  std::string draw_name_;

  void CallbackBtnQuit(Fl_Widget *);
  void CallbackBtnPNG(Fl_Widget *);
  void CallbackBtnSVG(Fl_Widget *);
  void CallbackBtnEPS(Fl_Widget *);
  void CallbackBtnPDF(Fl_Widget *);

  void SetupWindowElements();

private:
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
} // namespace librav

#endif /* CAIRO_WINDOW_HPP */
