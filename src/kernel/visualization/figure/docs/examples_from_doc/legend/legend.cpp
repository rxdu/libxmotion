# include "../prepare_data.hpp"
# include <figure/figure.hpp>

int main() {
  vec_t x,y;
  prepare_data(x, y, 4);

  mgl::Figure f,g;
  f.title("Default legend");
  f.plot(x, y).label("My Function");
  f.legend();
  f.save("legend_1");

  g.title("Manual positioning");
  g.plot(x, y).label("My Function");
  g.legend(0.5, 0.25);
  g.save("legend_2");

  return 0;
}
