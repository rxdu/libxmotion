/*
 * rect_grid_base_tiles.hpp
 *
 * Created on: Mar 29, 2018 11:19
 * Description:
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef RECT_GRID_BASE_TILES_HPP
#define RECT_GRID_BASE_TILES_HPP

#include <cstdint>
#include <vector>

namespace robosw {
// Reference:
// https://stackoverflow.com/questions/25492589/can-i-use-sfinae-to-selectively-define-a-member-variable-in-a-template-class?utm_medium=organic&utm_source=google_rich_qa&utm_campaign=google_rich_qa
template <typename T, typename IsScalar = void>
class GridTiles;

template <typename T>
class GridTiles<T, std::enable_if_t<std::is_floating_point<T>::value ||
                                    std::is_integral<T>::value>> {
 protected:
  GridTiles(int64_t x, int64_t y) { grid_tiles_ = Eigen::MatrixXd::Zero(y, x); }

  Eigen::MatrixXd grid_tiles_;

  void ResizeGridTiles(int64_t x, int64_t y) {
    Eigen::MatrixXd old_matrix = grid_tiles_;
    int64_t old_rows = grid_tiles_.rows();
    int64_t old_cols = grid_tiles_.cols();

    grid_tiles_.conservativeResize(y, x);
    grid_tiles_ = Eigen::MatrixXd::Zero(y, x);

    int64_t overlap_rows, overlap_cols;
    if (x > old_cols)
      overlap_cols = old_cols;
    else
      overlap_cols = x;

    if (y > old_rows)
      overlap_rows = old_rows;
    else
      overlap_rows = y;

    grid_tiles_.block(0, 0, overlap_rows, overlap_cols) =
        old_matrix.block(0, 0, overlap_rows, overlap_cols);
  }

  T GetTileAtCoordinate(int64_t x, int64_t y) const {
    return grid_tiles_(y, x);
  }

  T &GetTileRefAtCoordinate(int64_t x, int64_t y) { return grid_tiles_(y, x); }

  void SetTileAtCoordinate(int64_t x, int64_t y, T tile) {
    grid_tiles_(y, x) = tile;
  }
};

template <typename T>
class GridTiles<T, std::enable_if_t<!std::is_floating_point<T>::value &&
                                    !std::is_integral<T>::value>> {
 protected:
  GridTiles(int64_t x, int64_t y) { ResizeGridTiles(x, y); }

  std::vector<std::vector<T>> grid_tiles_;

  void ResizeGridTiles(int64_t x, int64_t y) {
    grid_tiles_.resize(y);
    for (auto &tile_array : grid_tiles_) tile_array.resize(x);
  }

  T GetTileAtCoordinate(int64_t x, int64_t y) const {
    return grid_tiles_[y][x];
  }

  template <typename T2 = T, typename std::enable_if<
                                 !std::is_pointer<T2>::value>::type * = nullptr>
  T &GetTileRefAtCoordinate(int64_t x, int64_t y) {
    return grid_tiles_[y][x];
  }

  void SetTileAtCoordinate(int64_t x, int64_t y, T tile) {
    grid_tiles_[y][x] = tile;
  }
};
}  // namespace robosw

#endif /* RECT_GRID_BASE_TILES_HPP */
