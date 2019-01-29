#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include "figure.hpp"

int main() {

  std::vector<double> x(100);
  std::vector<double> y1(100);
  Eigen::RowVectorXd y2(100);
  Eigen::VectorXd y3(100);

  for(int i = 0; i < 100; i++) {
    x[i] = i/100.;
    y1[i] = std::exp(-x[i]);
    y2(i) = std::exp(-1.5*x[i]);
    y3(i) = std::exp(-2*x[i]);
  }

  mgl::Figure fig;
  fig.plot(x, y1, "g-").label("e^{-x}");
  fig.plot(x, y2, "r|").label("e^{-1.5x}");
  fig.plot(x, y3, "b-").label("e^{-2x}").width(3);
  fig.fplot("exp(-2.5*x)").label("e^{-2.5x}").style("u:2");

  fig.ylabel("y Axis");
  fig.setlog(false, true, false);
  fig.legend(1, 1);
  fig.xlabel("x Axis");
  fig.title("Sample plot");

  fig.save("plot.eps");
  return 0;
}
