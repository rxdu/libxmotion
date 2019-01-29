# include <Eigen/Dense>
# include <figure.hpp>

int main () {
  Eigen::VectorXd t = Eigen::VectorXd::LinSpaced(5, 0, 1),
                  y = t.cwiseProduct(t);

  mgl::Figure fig;
  fig.plot(t, y, " no");
  fig.fplot("x^2", "r");
  fig.save("plot.eps");

  return 0;
}
