/* 
 * cairo_canvas.cpp
 * 
 * Created on: Mar 16, 2018 16:52
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "cairodraw/cairo_canvas.hpp"

#include <cmath>
#include <iostream>
#include <functional>

#include <cairo-svg.h>
#include <cairo-ps.h>
#include <cairo-pdf.h>

#include <FL/fl_draw.H>

using namespace librav;

CairoCanvas::CairoCanvas(int32_t x, int32_t y, int32_t w, int32_t h) : Fl_Box(x, y, w, h)
{
}

cairo_surface_t *CairoCanvas::cairo_gui_surface_create(int wo, int ho)
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

void CairoCanvas::SaveToFile(std::string filename, FileType type, int32_t wpts, int32_t hpts)
{
    cairo_surface_t *surface;
    cairo_t *cr;

    int32_t draw_width, draw_height;
    if (wpts == -1 || hpts == -1)
    {
        draw_width = this->w();
        draw_height = this->h();
    }
    else
    {
        draw_width = wpts;
        draw_height = hpts;
    }

    switch (type)
    {
    case FileType::PNG:
        surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, draw_width, draw_height);
        break;
    case FileType::SVG:
        surface = cairo_svg_surface_create(filename.c_str(), draw_width, draw_height);
        break;
    case FileType::EPS:
        surface = cairo_ps_surface_create(filename.c_str(), draw_width, draw_height);
        cairo_ps_surface_set_eps(surface, 1);
        break;
    case FileType::PDF:
        surface = cairo_pdf_surface_create(filename.c_str(), draw_width, draw_height);
        break;
    }
    cr = cairo_create(surface);

    if (type == FileType::PNG)
        cairo_translate(cr, 0.5, 0.5); // for anti-aliasing
    cairo_set_source_rgb(cr, 0, 0, 0); // drawing color set to black

    cairo_callback_(cr, 0, 0, draw_width, draw_height);

    if (type == FileType::PNG)
        cairo_surface_write_to_png(surface, filename.c_str());
    else
        cairo_show_page(cr);

    cairo_destroy(cr);
    cairo_surface_destroy(surface);
}

void CairoCanvas::draw(void)
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
    cairo_callback_(cr, x(), y(), w(), h());

    // release the cairo context
    cairo_destroy(cr);
    cairo_surface_destroy(surface);

    // remove clip regions
    fl_pop_clip(); // local clip region
    fl_pop_clip(); // "no_clip" region
}
