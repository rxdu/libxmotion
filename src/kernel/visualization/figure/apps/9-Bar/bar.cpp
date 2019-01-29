# include <Eigen/Dense>
# include <figure/figure.hpp>

int main() {
  const unsigned N = 10;
  Eigen::MatrixXd A(N, 3);
  A.col(0) = -Eigen::VectorXd::LinSpaced(N, 0, N);
  A.col(1) = Eigen::VectorXd::Random(N);
  A.col(2) = Eigen::VectorXd::LinSpaced(N, -1, 1);

  mgl::Figure fig;
  fig.bar(A);
  fig.save("data");
  return 0;
}
