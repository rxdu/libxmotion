
#include <cmath>

#include <FL/Fl.H>
#include <FL/Fl_Group.H>
#include <FL/Fl_Double_Window.H>
#include <FL/x.H>
#include <FL/Fl_Button.H>

#include "CairoBox.h"

Fl_Double_Window *win = (Fl_Double_Window *)0;
CairoBox *canvas = (CairoBox *)0;

const int wpts = 175; // width in points
const int hpts = 175; // height in points
const int wpix = 175; // width in pixels
const int hpix = 175; // height in pixels

void star(cairo_t *cr, double radius)
{
  // double theta = 0.8 * M_PI;
  // cairo_save(cr);
  // cairo_move_to(cr, 0.0, -radius);
  // for (int i = 0; i < 5; i++)
  // {
  //   cairo_rotate(cr, theta);
  //   cairo_line_to(cr, 0.0, -radius);
  // }
  // cairo_fill(cr);
  // cairo_restore(cr);
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
}

void CairoBox::graphic(cairo_t *cr, double x, double y, double w, double h)
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

static void cb_Quit(Fl_Button *, void *)
{
  win->hide();
}

static void cb_to_png(Fl_Button *, void *)
{
  char filename[] = "pngtest.png";
  fprintf(stderr, "Output in %s\n", filename);
  canvas->out_png(filename, wpix, hpix);
  return;
}

static void cb_to_svg(Fl_Button *, void *)
{
  char filename[] = "svgtest.svg";
  fprintf(stderr, "Output in %s\n", filename);
  canvas->out_svg(filename, wpts, hpts);
  return;
}

static void cb_to_eps(Fl_Button *, void *)
{
  char filename[] = "epstest.eps";
  fprintf(stderr, "Output in %s\n", filename);
  canvas->out_eps(filename, wpts, hpts);
  return;
}

static void cb_to_pdf(Fl_Button *, void *)
{
  char filename[] = "pdftest.pdf";
  fprintf(stderr, "Output in %s\n", filename);
  canvas->out_pdf(filename, wpts, hpts);
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

  win = new Fl_Double_Window(w, h);
  win->begin();
  canvas = new CairoBox(sp, sp, w - 2 * sp, h - 3 * sp - bh);
  buttons = new Fl_Group(sp, h - sp - bh, w - 2 * sp, bh);
  win->end();

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

  quit->callback((Fl_Callback *)cb_Quit);
  png->callback((Fl_Callback *)cb_to_png);
  svg->callback((Fl_Callback *)cb_to_svg);
  eps->callback((Fl_Callback *)cb_to_eps);
  pdf->callback((Fl_Callback *)cb_to_pdf);

  canvas->box(FL_FLAT_BOX);

  buttons->resizable(spacer);

  win->resizable(canvas);
  win->label("Cairo Graphics in FLTK with File Exports");
  win->show(argc, argv);

  return Fl::run();
}
