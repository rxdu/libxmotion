/* 
 * square_grid.hpp
 * 
 * Created on: Mar 28, 2018 23:07
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef SQUARE_GRID_HPP
#define SQUARE_GRID_HPP

#include <cstdint>
#include <vector>

#include "decomp/details/grid_base.hpp"

namespace librav
{
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

////////////////////////////////////////////////////////////////////

enum class SquareCellLabel
{
  OCCUPIED,
  FREE
};

struct GridPoint
{
  GridPoint(double xval = 0, double yval = 0) : x(xval), y(yval){};

  double x;
  double y;
};

////////////////////////////////////////////////////////////////////

template <typename AttributeType>
struct SquareCellBase
{
  SquareCellBase(int32_t xval, int32_t yval, int64_t idval = -1) : x(xval),
                                                                   y(yval),
                                                                   id(idval) {}

  // for easy reference, maybe unnecessary for some applications
  int64_t id = -1;

  // topological attributes
  int32_t x;
  int32_t y;
  SquareCellLabel label = SquareCellLabel::FREE;

  // geometrical attributes
  // 4 vertices in the order:
  // 0 - top left, 1 - top right
  // 2 - bottom left, 3 - bottom right
  GridPoint vertices[4];
  GridPoint center;

  // other application specific attributes
  // you can assign values to "cost_map" for visualization
  double cost_map = 0.0;

  AttributeType attribute;

  inline int64_t GetUniqueID() const { return id; }

  inline void UpdateGeometry(double size)
  {
    vertices[0].x = size * x;
    vertices[0].y = size * y;

    vertices[1].x = size * (x + 1);
    vertices[1].y = size * y;

    vertices[2].x = size * x;
    vertices[2].y = size * (y + 1);

    vertices[3].x = size * (x + 1);
    vertices[3].y = size * (y + 1);

    center.x = vertices[0].x + size / 2.0;
    center.y = vertices[0].y + size / 2.0;
  }

  inline void Print() const
  {
    std::cout << "cell " << id << " : " << x << " , " << y << " ; center : " << center.x << " , " << center.y << std::endl;
  }
};

////////////////////////////////////////////////////////////////////

template <typename T>
class SquareGridBase : public GridBase<SquareCellBase<T> *>
{
public:
  SquareGridBase(int32_t size_x, int32_t size_y, double cell_size = 0.1) : GridBase<SquareCellBase<T> *>(size_x, size_y),
                                                                           cell_size_(cell_size)
  {
    assert((size_x > 0 && size_y > 0));

    for (int32_t y = 0; y < size_y; y++)
      for (int32_t x = 0; x < size_x; x++)
      {
        SquareCellBase<T> *new_cell = new SquareCellBase<T>(x, y, CoordinateToID(x, y));
        new_cell->UpdateGeometry(cell_size);
        GridBase<SquareCellBase<T> *>::SetTileAtRawCoordinate(x, y, new_cell);
      }
  }
  SquareGridBase(const Eigen::MatrixXd &matrix, int32_t side_length, double cell_size = 0.1) : GridBase<SquareCellBase<T> *>(0, 0),
                                                                                               cell_size_(cell_size)
  {
    // determine size of grid
    int32_t center_x = matrix.cols() / 2;
    int32_t center_y = matrix.rows() / 2;

    int32_t grid_size_x, grid_size_y;
    bool shrink_x = false;
    bool shrink_y = false;
    if (matrix.cols() % side_length != 0)
      shrink_x = true;
    if (matrix.rows() % side_length != 0)
      shrink_y = true;
    grid_size_x = (center_x / side_length) * 2;
    grid_size_y = (center_y / side_length) * 2;

    Eigen::MatrixXd occupancy_matrix;
    if (shrink_x || shrink_y)
    {
      occupancy_matrix = Eigen::MatrixXd::Ones(grid_size_y * side_length, grid_size_x * side_length);

      int32_t x_start = 0, y_start = 0;
      if (shrink_x)
        x_start = center_x % side_length;
      if (shrink_y)
        y_start = center_y % side_length;

      occupancy_matrix = matrix.block(y_start, x_start, occupancy_matrix.rows(), occupancy_matrix.cols());
    }
    else
    {
      occupancy_matrix = matrix;
    }

    // create new grid
    GridBase<SquareCellBase<T> *>::ResizeGrid(grid_size_x, grid_size_y);
    for (int32_t y = 0; y < grid_size_y; y++)
      for (int32_t x = 0; x < grid_size_x; x++)
      {
        SquareCellBase<T> *new_cell = new SquareCellBase<T>(x, y, CoordinateToID(x, y));
        new_cell->UpdateGeometry(cell_size);
        GridBase<SquareCellBase<T> *>::SetTileAtRawCoordinate(x, y, new_cell);
      }

    // determine occupancy of grid
    int32_t half_size_x = grid_size_x / 2;
    int32_t half_size_y = grid_size_y / 2;
    for (int64_t x = 0; x < GridBase<SquareCellBase<T> *>::SizeX(); ++x)
      for (int64_t y = 0; y < GridBase<SquareCellBase<T> *>::SizeY(); ++y)
      {
        bool occupied = false;
        int32_t xmin = x * side_length;
        int32_t xmax = xmin + side_length;
        int32_t ymin = y * side_length;
        int32_t ymax = ymin + side_length;
        for (int i = xmin; i < xmax; ++i)
        {
          for (int j = ymin; j < ymax; ++j)
          {
            if (occupancy_matrix(j, i) != 0)
            {
              SetCellLabel(x, y, SquareCellLabel::OCCUPIED);
              occupied = true;
              break;
            }
          }
          if (occupied)
            break;
        }
      }
  }

  ~SquareGridBase()
  {
    for (int32_t y = 0; y < GridBase<SquareCellBase<T> *>::size_y_; y++)
      for (int32_t x = 0; x < GridBase<SquareCellBase<T> *>::size_x_; x++)
        delete GridBase<SquareCellBase<T> *>::GetTileAtRawCoordinate(x, y);
  }

  double GetCellSize() const { return cell_size_; }

  void SetCellLabel(int32_t x, int32_t y, SquareCellLabel label)
  {
    GridBase<SquareCellBase<T> *>::GetTileAtGridCoordinate(x, y)->label = label;
  }
  void SetCellLabel(int64_t id, SquareCellLabel label)
  {
    auto coordinate = IDToCoordinate(id);
    GridBase<SquareCellBase<T> *>::GetTileAtGridCoordinate(coordinate.GetX(), coordinate.GetY())->label = label;
  }

  GridCoordinate GetCoordinateFromID(int64_t id)
  {
    return IDToCoordinate(id);
  }

  int64_t GetIDFromCoordinate(int32_t x, int32_t y)
  {
    return CoordinateToID(x, y);
  }

  SquareCellBase<T> *GetCell(int64_t id)
  {
    auto coordinate = IDToCoordinate(id);
    return GridBase<SquareCellBase<T> *>::GetTileAtGridCoordinate(coordinate.GetX(), coordinate.GetY());
  }
  SquareCellBase<T> *GetCell(int32_t x, int32_t y)
  {
    return GridBase<SquareCellBase<T> *>::GetTileAtGridCoordinate(x, y);
  }

  std::vector<SquareCellBase<T> *> GetNeighbours(int32_t x, int32_t y, bool allow_diag)
  {
    std::vector<GridCoordinate> candidates;
    if (allow_diag)
    {
      for (int32_t xi = x - 1; xi <= x + 1; ++xi)
        for (int32_t yi = y - 1; yi <= y + 1; ++yi)
        {
          if (xi == x && yi == y)
            continue;
          candidates.emplace_back(xi, yi);
        }
    }
    else
    {
      candidates.emplace_back(x, y + 1);
      candidates.emplace_back(x, y - 1);
      candidates.emplace_back(x + 1, y);
      candidates.emplace_back(x - 1, y);
    }

    std::vector<SquareCellBase<T> *> neighbours;
    for (auto &can : candidates)
    {
      int32_t xi = can.GetX();
      int32_t yi = can.GetY();
      if (xi >= 0 && xi < GridBase<SquareCellBase<T> *>::size_x_ && yi >= 0 && yi < GridBase<SquareCellBase<T> *>::size_y_)
        neighbours.push_back(GridBase<SquareCellBase<T> *>::GetTileAtRawCoordinate(xi, yi));
    }

    return neighbours;
  }

  std::vector<SquareCellBase<T> *> GetNeighbours(int64_t id, bool allow_diag = true)
  {
    auto coordinate = IDToCoordinate(id);
    return GetNeighbours(coordinate.GetX(), coordinate.GetY(), allow_diag);
  }

private:
  double cell_size_;

  // CoordinateToID() and IDToCoordinate() are the only places that define
  //  the mapping between coordinate and id
  inline int64_t CoordinateToID(int32_t x, int32_t y)
  {
    return y * GridBase<SquareCellBase<T> *>::size_x_ + x;
  }

  inline GridCoordinate IDToCoordinate(int64_t id)
  {
    return GridCoordinate(id % GridBase<SquareCellBase<T> *>::size_x_, id / GridBase<SquareCellBase<T> *>::size_x_);
  }
};

using SquareCell = SquareCellBase<double>;
using SquareGrid = SquareGridBase<double>;
}

#endif /* SQUARE_GRID_HPP */
