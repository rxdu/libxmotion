#include <cmath>
#include <iostream>

#include "fastdraw/cairo_draw.hpp"

using namespace librav;

void star(cairo_t *cr, double radius)
{
    double theta = 0.8 * M_PI;
    cairo_save(cr);
    cairo_move_to(cr, 0.0, -radius);
    for (int i = 0; i < 5; i++)
    {
        cairo_rotate(cr, theta);
        cairo_line_to(cr, 0.0, -radius);
    }
    cairo_fill(cr);
    cairo_restore(cr);
}

void DrawStar(cairo_t *cr, double x, double y, double w, double h)
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

void DrawArc(cairo_t *cr, double x, double y, double w, double h)
{
    double xc = 128.0;
    double yc = 128.0;
    double radius = 100.0;
    double angle1 = 45.0 * (M_PI / 180.0);  /* angles are specified */
    double angle2 = 180.0 * (M_PI / 180.0); /* in radians           */

    cairo_set_line_width(cr, 10.0);
    cairo_arc_negative(cr, xc, yc, radius, angle1, angle2);
    cairo_stroke(cr);

    /* draw helping lines */
    cairo_set_source_rgba(cr, 1, 0.2, 0.2, 0.6);
    cairo_set_line_width(cr, 6.0);

    cairo_arc(cr, xc, yc, 10.0, 0, 2 * M_PI);
    cairo_fill(cr);

    cairo_arc(cr, xc, yc, radius, angle1, angle1);
    cairo_line_to(cr, xc, yc);
    cairo_arc(cr, xc, yc, radius, angle2, angle2);
    cairo_line_to(cr, xc, yc);
    cairo_stroke(cr);
}

int DrawLines(cairo_t *cr, double x, double y, double w, double h)
{
    cairo_set_line_width(cr, 30.0);
    cairo_set_line_cap(cr, CAIRO_LINE_CAP_BUTT); /* default */
    cairo_move_to(cr, 64.0, 50.0);
    cairo_line_to(cr, 64.0, 200.0);
    cairo_stroke(cr);
    cairo_set_line_cap(cr, CAIRO_LINE_CAP_ROUND);
    cairo_move_to(cr, 128.0, 50.0);
    cairo_line_to(cr, 128.0, 200.0);
    cairo_stroke(cr);
    cairo_set_line_cap(cr, CAIRO_LINE_CAP_SQUARE);
    cairo_move_to(cr, 192.0, 50.0);
    cairo_line_to(cr, 192.0, 200.0);
    cairo_stroke(cr);

    /* draw helping lines */
    cairo_set_source_rgb(cr, 1, 0.2, 0.2);
    cairo_set_line_width(cr, 2.56);
    cairo_move_to(cr, 64.0, 50.0);
    cairo_line_to(cr, 64.0, 200.0);
    cairo_move_to(cr, 128.0, 50.0);
    cairo_line_to(cr, 128.0, 200.0);
    cairo_move_to(cr, 192.0, 50.0);
    cairo_line_to(cr, 192.0, 200.0);
    cairo_stroke(cr);
}

int DrawExpt(cairo_t *cr, double x, double y, double w, double h)
{
    cairo_set_line_width(cr, 30.0);
    cairo_set_line_cap(cr, CAIRO_LINE_CAP_BUTT); /* default */
    cairo_move_to(cr, 64.0, 50.0);
    cairo_line_to(cr, 64.0, 200.0);
    cairo_stroke(cr);

    cairo_set_line_cap(cr, CAIRO_LINE_CAP_ROUND);
    cairo_move_to(cr, 128.0, 50.0);
    cairo_line_to(cr, 128.0, 200.0);
    cairo_stroke(cr);

    cairo_set_line_cap(cr, CAIRO_LINE_CAP_SQUARE);
    cairo_move_to(cr, 192.0, 50.0);
    cairo_line_to(cr, 192.0, 200.0);
    cairo_stroke(cr);

    /* draw helping lines */
    cairo_set_source_rgb(cr, 1, 0.2, 0.2);
    cairo_set_line_width(cr, 2.56);
    cairo_move_to(cr, 64.0, 50.0);
    cairo_line_to(cr, 64.0, 200.0);
    cairo_move_to(cr, 128.0, 50.0);
    cairo_line_to(cr, 128.0, 200.0);
    cairo_move_to(cr, 192.0, 50.0);
    cairo_line_to(cr, 192.0, 200.0);
    cairo_stroke(cr);
}

int DrawExpt2(cairo_t *cr, double x, double y, double w, double h)
{
    double xc = 128.0;
    double yc = 128.0;
    double radius = 100.0;
    double angle1 = 45.0 * (M_PI / 180.0);  /* angles are specified */
    double angle2 = 180.0 * (M_PI / 180.0); /* in radians           */

    cairo_set_line_width(cr, 10.0);
    cairo_arc_negative(cr, xc, yc, radius, angle1, angle2);
    cairo_stroke(cr);

    /* draw helping lines */
    cairo_set_source_rgba(cr, 1, 0.2, 0.2, 0.6);
    cairo_set_line_width(cr, 6.0);

    cairo_arc(cr, xc, yc, 10.0, 0, 2 * M_PI);
    cairo_fill(cr);

    cairo_arc(cr, xc, yc, radius, angle1, angle1);
    cairo_line_to(cr, xc, yc);
    cairo_arc(cr, xc, yc, radius, angle2, angle2);
    cairo_line_to(cr, xc, yc);
    cairo_stroke(cr);
}

int main()
{
    FastDraw::ShowCairo(DrawExpt2);

    return 0;
}