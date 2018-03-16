/* 
 * fltk_utils.hpp
 * 
 * Created on: Mar 16, 2018 16:51
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef FLTK_UTILS_HPP
#define FLTK_UTILS_HPP

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
class FltkCanvas : public Fl_Box
{
    void draw(void);
    cairo_t *cr;
    cairo_surface_t *surface;
    cairo_surface_t *set_surface(int wo, int ho);

  public:
    virtual void graphic(cairo_t *cr, double, double, double, double);
    void out_png(const char *filename, int, int);
    void out_svg(const char *filename, int, int);
    void out_eps(const char *filename, int, int);
    void out_pdf(const char *filename, int, int);

    void star(cairo_t *cr, double radius);

    FltkCanvas(int x, int y, int w, int h);
};

class FltkWindow
{
  public:
    FltkWindow() = default;
    ~FltkWindow() = default;

    void SetupWindow();

    const int w = 700;
    const int h = 600;
    const int sp = 5;
    const int bw = 75;
    const int bh = 25;

    const int wpts = 175; // width in points
    const int hpts = 175; // height in points
    const int wpix = 175; // width in pixels
    const int hpix = 175; // height in pixels

    Fl_Group *buttons;
    Fl_Button *quit;
    Fl_Button *png;
    Fl_Button *svg;
    Fl_Button *eps;
    Fl_Button *pdf;
    Fl_Box *spacer;
    FltkCanvas *canvas;
    Fl_Double_Window *win;

    void cb_Quit(Fl_Button *, void *);
    void cb_to_png(Fl_Button *, void *);
    void cb_to_svg(Fl_Button *, void *);
    void cb_to_eps(Fl_Button *, void *);
    void cb_to_pdf(Fl_Button *, void *);

    void run();
};
}

#endif /* FLTK_UTILS_HPP */
