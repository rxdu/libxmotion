/*
 * file_io_impl.hpp
 *
 * Created on: Jan 25, 2021 21:40
 * Description:
 *
 * Reference:
 * [1]
 * https://docs.opencv.org/2.4/modules/imgproc/doc/miscellaneous_transformations.html#cvtcolor
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef FILE_IO_IMPL_HPP
#define FILE_IO_IMPL_HPP

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

#include <boost/filesystem.hpp>

#define STB_IMAGE_IMPLEMENTATION
#include "file_io/details/stb/stb_image.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "file_io/details/stb/stb_image_write.h"

namespace xmotion {
template <typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
bool FileIO::ReadFromFile(
    std::string filename,
    Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime> &m) {
  // check file
  std::ifstream input(filename.c_str());
  if (input.fail()) {
    std::cerr << "Erro: failed to find file - " << filename << std::endl;
    m = Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime>(0, 0);
    return false;
  }

  // read lines and save to vector
  std::vector<Scalar> data;
  std::size_t row_num = 0;
  Scalar value;

  std::string line;
  while (std::getline(input, line)) {
    ++row_num;
    std::istringstream input_line(line);
    while (!input_line.eof()) {
      input_line >> value;
      data.push_back(value);
    }
  }
  input.close();

  // fill data from vector to matrix
  std::size_t col_num = data.size() / row_num;
  m = Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime>(row_num,
                                                                  col_num);
  for (std::size_t i = 0; i < row_num; i++) {
    for (std::size_t j = 0; j < col_num; j++) {
      m(i, j) = data[i * col_num + j];
    }
  }

  return true;
}

template <typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
bool FileIO::WriteToFile(
    std::string directory, std::string filename,
    Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime> matrix,
    bool overwrite) {
  // check directory existence: create the directory if non-existent
  if (directory.empty()) return false;
  if (!boost::filesystem::exists(directory)) {
    if (!boost::filesystem::create_directories(directory)) {
      std::cerr << "Error: directory is not found and failed to be created - "
                << directory << std::endl;
      return false;
    }
  }

  // get full file name and check file existence
  filename = directory + "/" + filename;
  if (boost::filesystem::exists(filename)) {
    if (!overwrite) {
      // File exists, but overwriting is not allowed. Abort.
      std::cerr << "Error: file already exists - " << filename << std::endl;
      return false;
    }
  }

  // now try to open file and write data to it
  std::ofstream file;
  file.open(filename.c_str());
  if (!file.is_open()) {
    std::cerr << "Error: failed to open file " << filename << std::endl;
    return false;
  }

  file << std::fixed;
  file << matrix;
  file.close();

  return true;
}

template <typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
bool FileIO::ReadFromImage(
    std::string filename,
    Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime> &m) {
  // check file
  std::ifstream input(filename.c_str());
  if (input.fail()) {
    std::cerr << "Erro: failed to find file - " << filename << std::endl;
    m = Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime>(0, 0);
    return false;
  }

  // read data using stb
  int x, y, n;
  unsigned char *data = stbi_load(filename.c_str(), &x, &y, &n, 0);
  if (data == NULL) {
    m = Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime>(0, 0);
    return false;
  }

  std::size_t col_num = x;
  std::size_t row_num = y;
  m = Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime>(row_num,
                                                                  col_num);

  std::cout << "read image size (w,y,d): " << x << " , " << y << " , " << n
            << std::endl;

  if (n <= 2) {
    for (int32_t j = 0; j < y; ++j) {
      for (int32_t i = 0; i < x; ++i) {
        m(j, i) = data[j * (x * n) + i * n];
      }
    }
  } else {
    for (int32_t j = 0; j < y; ++j) {
      for (int32_t i = 0; i < x; ++i) {
        m(j, i) = 0.299 * data[j * (x * n) + i * n] +
                  0.587 * data[j * (x * n) + i * n + 1] +
                  0.114 * data[j * (x * n) + i * n + 2];
      }
    }
  }

  stbi_image_free(data);

  return true;
}

template <typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
bool FileIO::WriteToImage(
    std::string directory, std::string filename,
    Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime> matrix,
    bool overwrite) {
  // check directory existence: create the directory if non-existent
  if (directory.empty()) return false;
  if (!boost::filesystem::exists(directory)) {
    if (!boost::filesystem::create_directories(directory)) {
      std::cerr << "Error: directory is not found and failed to be created - "
                << directory << std::endl;
      return false;
    }
  }

  // get full file name and check file existence
  filename = directory + "/" + filename;
  if (boost::filesystem::exists(filename)) {
    if (!overwrite) {
      // File exists, but overwriting is not allowed. Abort.
      std::cerr << "Error: file already exists - " << filename << std::endl;
      return false;
    }
  }

  // identify image type
  bool found_jpg_suffix_in_name =
      (filename.find(".jpg") != std::string::npos) ||
      (filename.find(".JPG") != std::string::npos);
  bool found_png_suffix_in_name =
      (filename.find(".png") != std::string::npos) ||
      (filename.find(".PNG") != std::string::npos);

  int64_t size = matrix.cols() * matrix.rows() * 1;
  int8_t *data = (int8_t *)malloc(size * sizeof(int8_t));
  for (int32_t j = 0; j < matrix.rows(); ++j)
    for (int32_t i = 0; i < matrix.cols(); ++i)
      data[j * matrix.cols() + i] = matrix(j, i);

  bool result;
  if (found_jpg_suffix_in_name) {
    result = stbi_write_jpg(filename.c_str(), matrix.cols(), matrix.rows(), 1,
                            data, 100);
  } else {
    if (!found_png_suffix_in_name) filename += ".png";
    result = stbi_write_png(filename.c_str(), matrix.cols(), matrix.rows(), 1,
                            data, matrix.cols());
  }
  free(data);

  return result;
}
}  // namespace xmotion

#endif /* FILE_IO_IMPL_HPP */
