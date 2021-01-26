/*
 * file_io_impl.hpp
 *
 * Created on: Jan 25, 2021 21:40
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef FILE_IO_IMPL_HPP
#define FILE_IO_IMPL_HPP

#include <string>
#include <fstream>
#include <iostream>

#include <boost/filesystem.hpp>

namespace robotnav {
template <typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
bool FileIO::ReadFromFile(
    std::string filename,
    Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime> &m) {
//   return DmpBbo::loadMatrix(filename, m);
    return false;
}

template <typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
bool FileIO::ReadFromImage(
    std::string file_name,
    Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime> &m);

template <typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
bool FileIO::WriteToFile(
    std::string directory, std::string filename,
    Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime> matrix,
    bool overwrite) {
  if (directory.empty()) return false;

  return true;
}

template <typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
bool FileIO::WriteToImage(
    std::string directory, std::string file_name,
    Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime> matrix,
    bool overwrite);
}  // namespace robotnav

#endif /* FILE_IO_IMPL_HPP */
