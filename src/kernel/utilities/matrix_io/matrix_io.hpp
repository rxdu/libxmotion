/* 
 * matrix_io.hpp
 * 
 * Created on: Nov 03, 2018 10:30
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef MATRIX_IO_HPP
#define MATRIX_IO_HPP

#include "EigenFileIO/EigenFileIO.hpp"

namespace librav
{
namespace MatrixIO
{
template <typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
bool LoadMatrix(std::string filename, Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime> &m)
{
    return DmpBbo::loadMatrix(filename, m);
}

template <typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
bool SaveMatrix(std::string filename, Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime> matrix, bool overwrite = false)
{
    return DmpBbo::saveMatrix(filename, matrix, overwrite);
}

template <typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
bool saveMatrix(std::string directory, std::string filename, Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime> matrix, bool overwrite = false)
{
    return DmpBbo::saveMatrix(directory, filename, matrix, overwrite);
}

} // namespace MatrixIO
} // namespace librav

#endif /* MATRIX_IO_HPP */
