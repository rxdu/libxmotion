/*
 * dense_grid.hpp
 *
 * Created on: Mar 28, 2018 23:25
 * Description:
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef DENSE_GRID_HPP
#define DENSE_GRID_HPP

#include <cstdint>
#include <vector>
#include <memory>

#include "decomp/details/rect_grid_base.hpp"
#include "decomp/square_grid.hpp"

namespace robosw {

/*
 * Coordinate System:
 *
 *		o --------------------> x
 *		|
 *		|
 *		|
 *		|
 *		|
 *		|
 *		v
 *		y
 */

class DenseGrid : public RectGridBase<double> {
 public:
  DenseGrid(int32_t size_x = 0, int32_t size_y = 0)
      : RectGridBase<double>(size_x, size_y){};
  ~DenseGrid() = default;

  inline void SetupGridWithMatrix(const Eigen::MatrixXd &mat) {
    ResizeGrid(mat.cols(), mat.rows());
    grid_tiles_ = mat;
  }

  inline void SetValueAtCoordinate(int64_t x, int64_t y, double val) {
    SetTileAtGridCoordinate(x, y, val);
  }

  inline double GetValueAtCoordinate(int64_t x, int64_t y) {
    return GetTileAtGridCoordinate(x, y);
  }

  // public functions inherited from RectGridBase
  /****************************************************************
  void ResizeGrid(int64_t x, int64_t y);
  void SetOriginCoordinate(int64_t origin_x, int64_t origin_y);
  *****************************************************************/

  inline void AddGrid(const DenseGrid &other) {
    assert(this->SizeX() == other.SizeX() && this->SizeY() == other.SizeY());
    this->grid_tiles_ += other.grid_tiles_;
  }

  inline void SubtractGrid(const DenseGrid &other) {
    assert(this->SizeX() == other.SizeX() && this->SizeY() == other.SizeY());
    this->grid_tiles_ -= other.grid_tiles_;
  }

  inline void AddGrid(DenseGrid *other) {
    assert(this->SizeX() == other->SizeX() && this->SizeY() == other->SizeY());
    this->grid_tiles_ += other->grid_tiles_;
  }

  inline void SubtractGrid(DenseGrid *other) {
    assert(this->SizeX() == other->SizeX() && this->SizeY() == other->SizeY());
    this->grid_tiles_ -= other->grid_tiles_;
  }

  inline Eigen::MatrixXd GetGridMatrix(bool normalize = false) const {
    Eigen::MatrixXd grid_matrix = grid_tiles_;
    if (normalize)
      grid_matrix = (grid_tiles_ + Eigen::MatrixXd::Ones(grid_tiles_.rows(),
                                                         grid_tiles_.cols()) *
                                       grid_tiles_.minCoeff()) /
                    (grid_tiles_.maxCoeff() - grid_tiles_.minCoeff()) * 1.0;
    return grid_matrix;
  }

  inline std::shared_ptr<SquareGrid> ConvertToSquareGrid(int32_t side_length,
                                                         double threshold = 0) {
    Eigen::MatrixXd matrix = GetGridMatrix(true);

    // determine size of grid
    int32_t center_x = matrix.cols() / 2;
    int32_t center_y = matrix.rows() / 2;

    int32_t grid_size_x, grid_size_y;
    bool shrink_x = false;
    bool shrink_y = false;
    if (matrix.cols() % side_length != 0) shrink_x = true;
    if (matrix.rows() % side_length != 0) shrink_y = true;
    grid_size_x = (center_x / side_length) * 2;
    grid_size_y = (center_y / side_length) * 2;

    Eigen::MatrixXd occupancy_matrix;
    if (shrink_x || shrink_y) {
      occupancy_matrix = Eigen::MatrixXd::Ones(grid_size_y * side_length,
                                               grid_size_x * side_length);

      int32_t x_start = 0, y_start = 0;
      if (shrink_x) x_start = center_x % side_length;
      if (shrink_y) y_start = center_y % side_length;

      occupancy_matrix = matrix.block(y_start, x_start, occupancy_matrix.rows(),
                                      occupancy_matrix.cols());
    } else {
      occupancy_matrix = matrix;
    }

    // create new grid
    auto grid = std::make_shared<SquareGrid>(grid_size_x, grid_size_y);

    // determine occupancy of grid
    int32_t half_size_x = grid_size_x / 2;
    int32_t half_size_y = grid_size_y / 2;
    for (int64_t x = 0; x < grid->SizeX(); ++x)
      for (int64_t y = 0; y < grid->SizeY(); ++y) {
        bool occupied = false;
        int32_t xmin = x * side_length;
        int32_t xmax = xmin + side_length;
        int32_t ymin = y * side_length;
        int32_t ymax = ymin + side_length;
        for (int i = xmin; i < xmax; ++i) {
          for (int j = ymin; j < ymax; ++j) {
            if (occupancy_matrix(j, i) > threshold) {
              grid->SetCellLabel(x, y, SquareCellLabel::OCCUPIED);
              occupied = true;
              break;
            }
          }
          if (occupied) break;
        }
      }

    return grid;
  }
};
}  // namespace robosw

#ifdef ENABLE_VISUAL
#include "details/dense_grid_visual.hpp"

namespace robosw {
void ShowDenseGridAsImage(const DenseGrid &grid, bool save_img = false,
                          std::string img_name = "DenseGrid") {
  ShowMatrixAsImage(grid.GetGridMatrix(true) * 128, img_name, save_img);
}

void ShowDenseGridAsColorMap(const DenseGrid &grid, bool save_img = false,
                             std::string img_name = "DenseGrid") {
  ShowMatrixAsColorMap(grid.GetGridMatrix(true), img_name, save_img);
}

void ShowPathOnDenseGrid(const DenseGrid &grid,
                         std::vector<RectGridIndex> waypoints,
                         bool save_img = false,
                         std::string img_name = "DenseGrid") {
  ShowPathOnMatrixAsColorMap(grid.GetGridMatrix(true), waypoints, img_name,
                             save_img);
}
}  // namespace robosw
#endif

#endif /* DENSE_GRID_HPP */
