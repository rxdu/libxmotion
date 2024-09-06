#include <iostream>

#include "math_utils/eigen_io.hpp"

using namespace xmotion;

int main() {
  //   Eigen::Matrix<double, 60, 60> matrix;
  Eigen::MatrixXd matrix;
  matrix.setZero(30, 30);

  for (int i = 0; i < 30; i++)
    for (int j = 0; j < 30; j++) {
      matrix(i, j) = i * j + 1;
    }

  std::string file_name = "matrix_file.data";

  EigenIO::WriteToFile(".", file_name, matrix);

  Eigen::MatrixXd md2;
  EigenIO::ReadFromFile("./" + file_name, md2);

  std::cout << "loaded:\n" << md2 << std::endl;

  Eigen::MatrixXd md3;
  EigenIO::ReadFromImage("./test.jpg", md3);

  //   std::cout << md3 << std::endl;

  EigenIO::WriteToImage(".", "test2.jpg", md3);

  return 0;
}