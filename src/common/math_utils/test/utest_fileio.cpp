/*
 * utest_polyline.cpp
 *
 * Created on: Nov 19, 2020 21:00
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#include "gtest/gtest.h"

#include "math_utils/eigen_io.hpp"

using namespace xmotion;

struct FileIOTest : testing::Test {
  FileIOTest() {
    matrix.setZero(5, 5);

    for (int i = 0; i < 5; i++)
      for (int j = 0; j < 5; j++) {
        matrix(i, j) = i * 5 + j;
      }
  }

  Eigen::MatrixXi matrix;
};

TEST_F(FileIOTest, RWFileTest) {
  std::string file_name = "matrix_file.data";
  EigenIO::WriteToFile(".", file_name, matrix);

  // test overwrite
  ASSERT_FALSE(EigenIO::WriteToFile(".", file_name, matrix));
  ASSERT_TRUE(EigenIO::WriteToFile(".", file_name, matrix, true));

  Eigen::MatrixXi md;
  EigenIO::ReadFromFile("./matrix_file.data", md);
  ASSERT_TRUE(md == matrix);
}

TEST_F(FileIOTest, RWImageJpgTest) {
  EigenIO::WriteToImage(".", "image_file.jpg", matrix);

  // test overwrite
  ASSERT_FALSE(EigenIO::WriteToImage(".", "image_file.jpg", matrix, false));
  ASSERT_TRUE(EigenIO::WriteToImage(".", "image_file.jpg", matrix));

  Eigen::MatrixXi md;
  EigenIO::ReadFromImage("./image_file.jpg", md);
  std::cout << "jpg: \n" << md << std::endl;
  //   ASSERT_TRUE(md == matrix);
}

TEST_F(FileIOTest, RWImagePngTest) {
  EigenIO::WriteToImage(".", "image_file.png", matrix);

  // test overwrite
  ASSERT_FALSE(EigenIO::WriteToImage(".", "image_file.png", matrix, false));
  ASSERT_TRUE(EigenIO::WriteToImage(".", "image_file.png", matrix));

  Eigen::MatrixXi md;
  EigenIO::ReadFromImage("./image_file.png", md);
  std::cout << "png: \n" << md << std::endl;
  ASSERT_TRUE(md == matrix);
}