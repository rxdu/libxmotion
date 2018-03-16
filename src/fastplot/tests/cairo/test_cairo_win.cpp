//
// "$Id: arc.cxx 5115 2006-05-12 16:00:00Z fabien $"
//
// Arc drawing test program for the Fast Light Tool Kit (FLTK).
//
// Copyright 1998-2010 by Bill Spitzak and others.
//
// This library is free software. Distribution and use rights are outlined in
// the file "COPYING" which should have been included with this file.  If this
// file is missing or damaged, see the license at:
//
//     http://www.fltk.org/COPYING.php
//
// Please report all bugs and problems on the following page:
//
//     http://www.fltk.org/str.php
//

#define FLTK_HAVE_CAIRO

#include <cairo-xlib.h>
#include <cairo/config.h>

#include <FL/Fl_Cairo_Window.H>
#include <FL/Fl_Box.H>
#include <FL/x.H>
#include <FL/fl_draw.H>
#include <FL/math.h>

#include <FL/Fl.H>
#include <FL/Fl_Group.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Button.H>

#define DEF_WIDTH 0.03

// uncomment the following line to enable cairo context autolink feature:
// #define AUTOLINK

// put your drawing stuff here
float drawargs[7] = {90, 90, 100, 100, 0, 360, 0};
const char *name[7] = {"X", "Y", "W", "H", "start", "end", "rotate"};

static void centered_text(cairo_t *cr, double x0, double y0, double w0, double h0, const char *my_text)
{
    cairo_select_font_face(cr, "Sans", CAIRO_FONT_SLANT_OBLIQUE, CAIRO_FONT_WEIGHT_BOLD);
    cairo_set_source_rgba(cr, 0.9, 0.9, 0.4, 0.6);
    cairo_text_extents_t extents;
    cairo_text_extents(cr, my_text, &extents);
    double x = (extents.width / 2 + extents.x_bearing);
    double y = (extents.height / 2 + extents.y_bearing);
    cairo_move_to(cr, x0 + w0 / 2 - x, y0 + h0 / 2 - y);
    cairo_text_path(cr, my_text);
    cairo_fill_preserve(cr);
    cairo_set_source_rgba(cr, 0, 0, 0, 1);
    cairo_set_line_width(cr, 0.004);
    cairo_stroke(cr);
    cairo_set_line_width(cr, DEF_WIDTH);
}

static void round_button(cairo_t *cr, double x0, double y0,
                         double rect_width, double rect_height, double radius,
                         double r, double g, double b)
{

    double x1, y1;

    x1 = x0 + rect_width;
    y1 = y0 + rect_height;
    if (!rect_width || !rect_height)
        return;
    if (rect_width / 2 < radius)
    {
        if (rect_height / 2 < radius)
        {
            cairo_move_to(cr, x0, (y0 + y1) / 2);
            cairo_curve_to(cr, x0, y0, x0, y0, (x0 + x1) / 2, y0);
            cairo_curve_to(cr, x1, y0, x1, y0, x1, (y0 + y1) / 2);
            cairo_curve_to(cr, x1, y1, x1, y1, (x1 + x0) / 2, y1);
            cairo_curve_to(cr, x0, y1, x0, y1, x0, (y0 + y1) / 2);
        }
        else
        {
            cairo_move_to(cr, x0, y0 + radius);
            cairo_curve_to(cr, x0, y0, x0, y0, (x0 + x1) / 2, y0);
            cairo_curve_to(cr, x1, y0, x1, y0, x1, y0 + radius);
            cairo_line_to(cr, x1, y1 - radius);
            cairo_curve_to(cr, x1, y1, x1, y1, (x1 + x0) / 2, y1);
            cairo_curve_to(cr, x0, y1, x0, y1, x0, y1 - radius);
        }
    }
    else
    {
        if (rect_height / 2 < radius)
        {
            cairo_move_to(cr, x0, (y0 + y1) / 2);
            cairo_curve_to(cr, x0, y0, x0, y0, x0 + radius, y0);
            cairo_line_to(cr, x1 - radius, y0);
            cairo_curve_to(cr, x1, y0, x1, y0, x1, (y0 + y1) / 2);
            cairo_curve_to(cr, x1, y1, x1, y1, x1 - radius, y1);
            cairo_line_to(cr, x0 + radius, y1);
            cairo_curve_to(cr, x0, y1, x0, y1, x0, (y0 + y1) / 2);
        }
        else
        {
            cairo_move_to(cr, x0, y0 + radius);
            cairo_curve_to(cr, x0, y0, x0, y0, x0 + radius, y0);
            cairo_line_to(cr, x1 - radius, y0);
            cairo_curve_to(cr, x1, y0, x1, y0, x1, y0 + radius);
            cairo_line_to(cr, x1, y1 - radius);
            cairo_curve_to(cr, x1, y1, x1, y1, x1 - radius, y1);
            cairo_line_to(cr, x0 + radius, y1);
            cairo_curve_to(cr, x0, y1, x0, y1, x0, y1 - radius);
        }
    }
    cairo_close_path(cr);

    cairo_pattern_t *pat =
        //cairo_pattern_create_linear (0.0, 0.0,  0.0, 1.0);
        cairo_pattern_create_radial(0.25, 0.24, 0.11, 0.24, 0.14, 0.35);
    cairo_pattern_set_extend(pat, CAIRO_EXTEND_REFLECT);

    cairo_pattern_add_color_stop_rgba(pat, 1.0, r, g, b, 1);
    cairo_pattern_add_color_stop_rgba(pat, 0.0, 1, 1, 1, 1);
    cairo_set_source(cr, pat);
    cairo_fill_preserve(cr);
    cairo_pattern_destroy(pat);

    //cairo_set_source_rgb (cr, 0.5,0.5,1);    cairo_fill_preserve (cr);
    cairo_set_source_rgba(cr, 0, 0, 0.5, 0.3);
    cairo_stroke(cr);

    cairo_set_font_size(cr, 0.08);
    centered_text(cr, x0, y0, rect_width, rect_height, "FLTK loves Cairo!");
}

// The cairo rendering cb called during Fl_Cairo_Window::draw() :
static void my_cairo_draw_cb(Fl_Cairo_Window *window, cairo_t *cr)
{

    int w = window->w(), h = window->h();

    cairo_set_line_width(cr, DEF_WIDTH);
    cairo_scale(cr, w, h);

    round_button(cr, 0.1, 0.05, 0.8, 0.2, 0.4, 0, 0, 1);
    // round_button(cr, 0.1, 0.35, 0.8, 0.2, 0.4, 1, 0, 0);
    round_button(cr, 0.1, 0.65, 0.8, 0.2, 0.4, 0, 1, 0);
    return;
}

int main(int argc, char **argv)
{
    const int w = 700;
    const int h = 600;
    const int sp = 5;
    const int bw = 75;
    const int bh = 25;

    Fl_Group *buttons;
    Fl_Button *quit;
    Fl_Button *png;
    Fl_Button *svg;
    Fl_Button *eps;
    Fl_Button *pdf;
    Fl_Box *spacer;

#ifdef AUTOLINK
    Fl::cairo_autolink_context(true);
#endif
    Fl_Cairo_Window window(300, 300);

    window.begin();
    buttons = new Fl_Group(sp, h - sp - bh, w - 2 * sp, bh);
    window.end();

    int x = sp;
    buttons->begin();
    quit = new Fl_Button(x, h - sp - bh, bw, bh, "Quit");
    x += bw;
    png = new Fl_Button(x, h - sp - bh, bw, bh, "To png");
    x += bw;
    svg = new Fl_Button(x, h - sp - bh, bw, bh, "To svg");
    x += bw;
    eps = new Fl_Button(x, h - sp - bh, bw, bh, "To eps");
    x += bw;
    pdf = new Fl_Button(x, h - sp - bh, bw, bh, "To pdf");
    x += bw;
    spacer = new Fl_Box(FL_NO_BOX, x, h - sp - bh, 1, bh, "");
    buttons->end();

    buttons->resizable(spacer);

    window.resizable(&window);
    window.color(FL_WHITE);
    window.set_draw_cb(my_cairo_draw_cb);
    window.show(argc, argv);

    return Fl::run();
}

//
// End of "$Id: arc.cxx 5115 2006-05-12 16:00:00Z fabien $".
//