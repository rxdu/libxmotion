/*
 * file_io.hpp
 *
 * Created on: Jan 25, 2021 21:25
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef FILE_IO_HPP
#define FILE_IO_HPP

#include "eigen3/Eigen/Core"

namespace xmotion {
namespace EigenIO {
template <typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
bool ReadFromFile(
    std::string filename,
    Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime> &m);

template <typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
bool WriteToFile(
    std::string directory, std::string filename,
    Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime> matrix,
    bool overwrite = false);

template <typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
bool ReadFromImage(
    std::string filename,
    Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime> &m);

template <typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
bool WriteToImage(
    std::string directory, std::string filename,
    Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime> matrix,
    bool overwrite = true);
}  // namespace EigenIO
}  // namespace xmotion

#include "details/eigen_io_impl.hpp"

#endif /* FILE_IO_HPP */
