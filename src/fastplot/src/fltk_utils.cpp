/* 
 * fltk_utils.cpp
 * 
 * Created on: Mar 16, 2018 16:52
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "fastplot/fltk_utils.hpp"

#include <cmath>
#include <functional>

#include <cairo-svg.h>
#include <cairo-ps.h>
#include <cairo-pdf.h>

#include <FL/fl_draw.H>

using namespace librav;

FltkCanvas::FltkCanvas(int x, int y, int w, int h) : Fl_Box(x, y, w, h)
{
}

void FltkCanvas::SetDrawCallback(CallBackFunc_t func)
{
    cairo_callback_ = func;
}

cairo_surface_t *FltkCanvas::cairo_gui_surface_create(int wo, int ho)
{
#ifdef WIN32
#warning win32 mode
    /* Get a Cairo surface for the current DC */
    HDC dc = fl_gc; /* Exported by fltk */
    return cairo_win32_surface_create(dc);
#elif defined(__APPLE__)
#warning Apple Quartz mode
    /* Get a Cairo surface for the current CG context */
    CGContext *ctx = fl_gc;
    return cairo_quartz_surface_create_for_cg_context(ctx, wo, ho);
#else
    /* Get a Cairo surface for the current display */
    return cairo_xlib_surface_create(fl_display, fl_window, fl_visual->visual, wo, ho);
#endif
}

void FltkCanvas::SaveToPNG(const char *filename, int wpix, int hpix)
{
    cairo_surface_t *surface;
    cairo_t *cr;

    // setup, see cairo_format_t for discussion of first argument
    surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, wpix, hpix);
    cr = cairo_create(surface);

    cairo_translate(cr, 0.5, 0.5);     // for anti-aliasing
    cairo_set_source_rgb(cr, 0, 0, 0); // drawing color set to black

    // virtual function
    graphic(cr, 0, 0, wpix, hpix);

    cairo_surface_write_to_png(surface, filename);
    cairo_destroy(cr);
    cairo_surface_destroy(surface);
}

void FltkCanvas::SaveToSVG(const char *filename, int wpts, int hpts)
{
    cairo_surface_t *surface;
    cairo_t *cr;

    surface = cairo_svg_surface_create(filename, wpts, hpts);
    cr = cairo_create(surface);
    cairo_set_source_rgb(cr, 0, 0, 0);

    // user coordinates -- see cairo_svg_surface_create
    graphic(cr, 0, 0, wpts, hpts);

    cairo_destroy(cr);
    cairo_surface_destroy(surface);
}

void FltkCanvas::SaveToEPS(const char *filename, int wpts, int hpts)
{
    cairo_surface_t *surface;
    cairo_t *cr;

    surface = cairo_ps_surface_create(filename, wpts, hpts);
    cairo_ps_surface_set_eps(surface, 1);
    cr = cairo_create(surface);
    cairo_set_source_rgb(cr, 0, 0, 0);

    // user coordinates -- see cairo_ps_surface_create
    graphic(cr, 0, 0, wpts, hpts);

    cairo_show_page(cr);
    cairo_destroy(cr);
    cairo_surface_destroy(surface);
    return;
}

void FltkCanvas::SaveToPDF(const char *filename, int wpts, int hpts)
{
    cairo_surface_t *surface;
    cairo_t *cr;

    surface = cairo_pdf_surface_create(filename, wpts, hpts);
    cr = cairo_create(surface);
    cairo_set_source_rgb(cr, 0, 0, 0);

    // user coordinates -- see cairo_pdf_surface_create
    graphic(cr, 0, 0, wpts, hpts);

    cairo_show_page(cr);
    cairo_destroy(cr);
    cairo_surface_destroy(surface);
    return;
}

void FltkCanvas::draw(void)
{
    // using fltk functions, set up white background with thin black frame
    fl_push_no_clip(); /* remove any clipping region set by the expose events... */
    fl_push_clip(x(), y(), w(), h());
    fl_color(FL_WHITE);
    fl_rectf(x(), y(), w(), h());
    fl_color(FL_BLACK);
    fl_rect(x(), y(), w(), h());

    // set up cairo structures
    cairo_surface_t *surface = cairo_gui_surface_create(w(), h());
    cairo_t *cr = cairo_create(surface);

    /* All Cairo co-ordinates are shifted by 0.5 pixels to correct anti-aliasing */
    cairo_translate(cr, 0.5, 0.5);
    cairo_set_source_rgb(cr, 0.0, 0.0, 0.0); // set drawing color to black
    cairo_new_path(cr);

    // virtual function defined in driver program
    graphic(cr, x(), y(), w(), h());

    // release the cairo context
    cairo_destroy(cr);
    cairo_surface_destroy(surface);

    // remove clip regions
    fl_pop_clip(); // local clip region
    fl_pop_clip(); // "no_clip" region
}

void FltkCanvas::star(cairo_t *cr, double radius)
{
    double theta = 0.8 * M_PI;
    cairo_save(cr);

    // cairo_move_to(cr, 0.0, -radius);
    // for (int i = 0; i < 5; i++)
    // {
    //   cairo_rotate(cr, theta);
    //   cairo_line_to(cr, 0.0, -radius);
    // }
    // cairo_fill(cr);

    double x = 25.6, y = 128.0;
    double x1 = 102.4, y1 = 230.4,
           x2 = 153.6, y2 = 25.6,
           x3 = 230.4, y3 = 128.0;

    cairo_move_to(cr, x, y);
    cairo_curve_to(cr, x1, y1, x2, y2, x3, y3);

    cairo_set_line_width(cr, 10.0);
    cairo_stroke(cr);

    cairo_set_source_rgba(cr, 1, 0.2, 0.2, 0.6);
    cairo_set_line_width(cr, 6.0);
    cairo_move_to(cr, x, y);
    cairo_line_to(cr, x1, y1);
    cairo_move_to(cr, x2, y2);
    cairo_line_to(cr, x3, y3);
    cairo_stroke(cr);

    cairo_restore(cr);
}

void FltkCanvas::graphic(cairo_t *cr, double x, double y, double w, double h)
{
    double f = 1.0 / (1.0 + sin(0.3 * M_PI));
    cairo_translate(cr, x + w / 2, y + f * h);
    double radius = f * h;
    double srink[] = {1.0, 0.95, 0.85, 0.75};
    for (int i = 0; i < 4; i++)
    {
        if (i % 2)
            cairo_set_source_rgb(cr, 0.0, 0.0, 0.5);
        else
            cairo_set_source_rgb(cr, 1.0, 1.0, 1.0);
        star(cr, srink[i] * radius);
    }
}

////////////////////////////////////////////////////////////////////////////////////

void FltkWindow::SetupWindow()
{
    win = new Fl_Double_Window(w, h);
    win->begin();
    canvas = new FltkCanvas(sp, sp, w - 2 * sp, h - 3 * sp - bh);
    button_group = new Fl_Group(sp, h - sp - bh, w - 2 * sp, bh);
    win->end();

    int x = sp;
    button_group->begin();
    png = new Fl_Button(x, h - sp - bh, bw, bh, "Save png");
    x += bw;
    svg = new Fl_Button(x, h - sp - bh, bw, bh, "Save svg");
    x += bw;
    eps = new Fl_Button(x, h - sp - bh, bw, bh, "Save eps");
    x += bw;
    pdf = new Fl_Button(x, h - sp - bh, bw, bh, "Save pdf");
    // x += bw;
    // quit = new Fl_Button(x, h - sp - bh, bw, bh, "Quit");
    button_group->end();

    // quit->callback(StaticCallback_BtnQuit, this);
    png->callback(StaticCallback_BtnPNG, this);
    svg->callback(StaticCallback_BtnSVG, this);
    eps->callback(StaticCallback_BtnEPS, this);
    pdf->callback(StaticCallback_BtnPDF, this);

    canvas->box(FL_FLAT_BOX);
    button_group->resizable(spacer);

    win->resizable(canvas);
    win->label("Cairo Graphics");
}

void FltkWindow::CallbackBtnQuit(Fl_Widget *)
{
    win->hide();
}

void FltkWindow::CallbackBtnPNG(Fl_Widget *)
{
    char filename[] = "pngtest.png";
    fprintf(stderr, "Output in %s\n", filename);
    canvas->SaveToPNG(filename, wpix, hpix);
    return;
}

void FltkWindow::CallbackBtnSVG(Fl_Widget *)
{
    char filename[] = "svgtest.svg";
    fprintf(stderr, "Output in %s\n", filename);
    canvas->SaveToSVG(filename, wpts, hpts);
    return;
}

void FltkWindow::CallbackBtnEPS(Fl_Widget *)
{
    char filename[] = "epstest.eps";
    fprintf(stderr, "Output in %s\n", filename);
    canvas->SaveToEPS(filename, wpts, hpts);
    return;
}

void FltkWindow::CallbackBtnPDF(Fl_Widget *)
{
    char filename[] = "pdftest.pdf";
    fprintf(stderr, "Output in %s\n", filename);
    canvas->SaveToPDF(filename, wpts, hpts);
    return;
}

void FltkWindow::run()
{
    win->show();

    Fl::run();
}