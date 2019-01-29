# include <figure/figure.hpp>

int main () {
  Eigen::VectorXd x = Eigen::VectorXd::LinSpaced(500, -1, 1),
                  y = (5*x).array().cos().matrix(),
                  u = Eigen::VectorXd::LinSpaced(500, 0, 2.3),
                  v = ((u.array()/5.).exp() + 3).matrix();

  mgl::Figure f,g,h;
  f.title("f(x) = cos(5x)");
  f.ranges(-1, 1, -1, 1);
  f.plot(x, y, "b");
  f.save("ranges_1");

  g.title("f(x) = exp(x/5) + 3");
  g.plot(u, v, "b");
  g.ranges(0, 2.3, 4, 5);
  g.save("ranges_2");

  h.title("Non positive data");
  h.ranges(-1, 1, 0, 5);
  h.setlog(true, true);
  h.plot(x, y);
  h.save("ranges_3");

  return 0;
}
