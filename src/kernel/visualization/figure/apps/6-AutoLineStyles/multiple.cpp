# include <Eigen/Dense>
# include "figure.hpp"

int main () {
  Eigen::VectorXd t = Eigen::VectorXd::LinSpaced(500, 0, 1),
                  y = t;

  mgl::Figure fig;
  for (unsigned i = 1; i < 15; ++i) {
    fig.plot(t, y);
    y = y.cwiseProduct(t);
  }
  fig.save("many.eps");

  mgl::Figure fig2;
  fig2.ranges(0, 10, 0, 16);
  fig2.fplot("0.2*x", "b0");
  fig2.fplot("0.4*x", "b1");
  fig2.fplot("0.6*x", "b2");
  fig2.fplot("0.8*x", "b3");
  fig2.fplot("x", "b4");
  fig2.fplot("1.2*x", "b5");
  fig2.fplot("1.4*x", "b6");
  fig2.fplot("1.6*x", "b7");
  fig2.save("manysame.eps");
  

  return 0;
}

