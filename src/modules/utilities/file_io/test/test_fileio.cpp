#include <iostream>

#include "file_io/file_io.hpp"

using namespace robotnav;

int main() {
  // Eigen::Matrix<double, 60, 60> matrix;
  // Eigen::MatrixXd matrix;
  // matrix.setZero(30, 30);

  // for (int i = 0; i < 30; i++)
  //     for (int j = 0; j < 30; j++)
  //         matrix(i, j) = i * j + 1;

  // DmpBbo::saveMatrix("matrix_file", matrix);

  Eigen::MatrixXd md2;
  FileIO::ReadFromFile("./matrix_file", md2);

  std::cout << "loaded:\n" << md2 << std::endl;

  return 0;
}