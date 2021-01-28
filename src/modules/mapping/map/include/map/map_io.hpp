/*
 * map_io.hpp
 *
 * Created on: Jan 25, 2021 21:06
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef MAP_IO_HPP
#define MAP_IO_HPP

#include <string>
#include <vector>

#include <eigen3/Eigen/Core>

namespace robotnav {
struct MapParameters {
  std::string image_file_name;
  double resolution{0};
  std::vector<double> origin{0, 0, 0};
  double free_thresh;
  double occupied_thresh;
  //   MapMode mode;
  bool negate;
};

template <typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
bool ReadMap(std::string filename,
             Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime> &m);

}  // namespace robotnav

#endif /* MAP_IO_HPP */
