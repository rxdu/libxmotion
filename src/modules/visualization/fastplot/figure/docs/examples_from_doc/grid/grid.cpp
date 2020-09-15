# include <figure/figure.hpp>
# include "../prepare_data.hpp"

int main () {
  vec_t x, y;
  prepare_data(x, y, 2);

  mgl::Figure f, g;
  f.title("Unset grid");
  f.grid(false);
  f.plot(x, y);
  f.save("grid_1.eps");

  g.title("Fine, grey grid");
  g.grid(true, "!", "h");
  g.plot(x, y);
  g.save("grid_2");

  return 0;
}

