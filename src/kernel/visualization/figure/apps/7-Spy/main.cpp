# include <vector>
# include <Eigen/Dense>
# include <Eigen/Sparse>
# include <figure/figure.hpp>

typedef Eigen::Triplet<double> T;

int main() {
  Eigen::MatrixXd D(100,200); 
  D.setZero();
  Eigen::SparseMatrix<double> S(100,200);
  std::vector<T> triplets;

  // some diagonal entries
  for (int i = 0; i < 100; ++i) {
    D(i,i) = 1;
    triplets.push_back( T(i, i, 1) );
  }

  // some off diagonal entries 
  for (int j = 0; j < 99; ++j) {
    D(j, j + 1) = -2;
    D(j + 1, j) = 5;

    triplets.push_back( T(j, j + 1, -2) );
    triplets.push_back( T(j + 1, j, 5) );
  }

  // another entry
  D(10, 50) = 0.21;
  triplets.push_back( T(10, 50, 0.21) );

  S.setFromTriplets(triplets.begin(), triplets.end());
  S.makeCompressed();

  mgl::Figure denseSpy, sparseSpy;
  denseSpy.spy( D );
  denseSpy.save("denseSpy");

  sparseSpy.spy( S );
  sparseSpy.save("sparseSpy");

  return 0;
}
