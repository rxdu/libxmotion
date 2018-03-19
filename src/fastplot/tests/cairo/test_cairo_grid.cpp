#include <cmath>
#include <iostream>

#include "fastplot/cairo_utils.hpp"

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

void DrawGrid(cairo_t *cr, double x, double y, double w, double h)
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

int main()
{
    CairoWindow win(DrawGrid);
    win.Run();

    return 0;
}