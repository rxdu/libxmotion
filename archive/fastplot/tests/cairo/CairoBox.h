
#ifndef CAIROBOX
#define CAIROBOX

#include <FL/Fl_Box.H>
#include <cairo.h>

#ifdef WIN32
#  include <cairo-win32.h>
#elif defined (__APPLE__)
#  include <cairo-quartz.h>
#else
#  include <cairo-xlib.h>
#endif

class CairoBox : public Fl_Box 
  {
  void draw(void); 
  cairo_t*          cr;
  cairo_surface_t*  surface;
  cairo_surface_t*  set_surface(int wo, int ho);
  public:
  virtual void graphic(cairo_t* cr, double, double, double, double);  
  void out_png(const char* filename, int, int);
  void out_svg(const char* filename, int, int);
  void out_eps(const char* filename, int, int);
  void out_pdf(const char* filename, int, int);
  CairoBox(int x, int y, int w, int h);
  };

#endif

