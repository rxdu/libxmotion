# include <figure/figure.hpp>

int main () {
  mgl::Figure f,g;
  f.fplot("3*x^2 - 4.5/x)*exp(-x/1.3)");
  f.fplot("5*sin(5*x)*exp(-x)", "r").label("5sin(5x)*e^{-x}");
  f.legend();
  f.ranges(0.5, 5, -5, 5);
  f.save("fplot_1");

  Eigen::VectorXd x = Eigen::VectorXd::LinSpaced(50, 1, 5),
                  y = x.cwiseProduct(x) + Eigen::VectorXd::Random(50).cwiseAbs();

  g.title("Runtimes");
  g.plot(x, y).label("Benchmark");
  g.fplot("x^2", "k;").label("\\ O(x^2)");
  g.legend();
  g.save("fplot_2");

  return 0;
}

