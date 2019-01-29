# include "../prepare_data.hpp"
# include <figure/figure.hpp>

int main () {
  vec_t x = vec_t::LinSpaced(500, 1, 5),
        y1 = x,
        y2 = x.cwiseProduct(x),
        y3 = x.array().exp().matrix();

  mgl::Figure f, g;
  f.title("Multiple setlog calls");
  f.setlog(true, false);
  f.plot(x, y1).label("f(x) = x");
  f.setlog(false, true);
  f.plot(x, y2).label("f(x) = x^2");
  f.setlog(true, true);
  f.plot(x, y3).label("f(x) = exp(x)");
  f.legend(0,1);
  f.save("setlog_1");

  g.title("Default scaling");
  g.plot(x, y1).label("f(x) = x");
  g.plot(x, y2).label("f(x) = x^2");
  g.plot(x, y3).label("f(x) = exp(x)");
  g.legend(0,1);
  g.save("setlog_2");

  return 0;
}
