# include "../prepare_data.hpp"
# include <figure/figure.hpp>

int main () {
  vec_t x, y;
  prepare_data(x, y, 3, 10);
  
  mgl::Figure f, g;
  f.plot(x, y, "g+");
  f.xlabel("Linear x axis");
  f.save("xlabel_1");

  g.plot(x, y, "g+");
  g.setlog(true, true);
  g.xlabel("Logarithmic x axis");
  g.save("xlabel_2");

  return 0;
}
