# include "../prepare_data.hpp"
# include <figure/figure.hpp>

int main () {
  vec_t x,y,x0,y0;
  prepare_data(x, y, 3);
  prepare_data(x0, y0, 3, 10);


  mgl::Figure f,g,h;
  f.title("Plot w/ dashed style");
  f.plot(x, y, "g;");
  f.save("plot_1");

  g.title("Default style");
  g.plot(x, y);
  g.save("plot_2");

  h.title("Data plotting");
  h.plot(x0, y0, " *r");
  h.save("plot_3");

  return 0;
}
