# include <Eigen/Dense>
# include <figure/figure.hpp>

int main () {
  Eigen::VectorXd t = Eigen::VectorXd::LinSpaced(500, 0, 50),
                  x = t.cwiseProduct(t.array().sin().matrix()),
                  y = t.cwiseProduct(t.array().cos().matrix()),
                  z = t;

  mgl::Figure f;
  f.title("x = t*sin(t), y = t*cos(t), z = t");
  f.plot3(x, y, z);
  f.save("plot3_1");

  return 0;
}
