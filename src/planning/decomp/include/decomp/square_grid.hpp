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
    std::cout << "cell " << id << " : " << x << " , " << y 
      << " ; center : " << center.x << " , " << center.y << std::endl;
  }
};

using SquareCell = SquareCellBase<double>;

////////////////////////////////////////////////////////////////////

template <typename T>
class SquareGridBase : public GridBase<SquareCellBase<T> *>
{
public:
  SquareGridBase(int32_t size_x, int32_t size_y, double cell_size = 0.1);
  SquareGridBase(const Eigen::MatrixXd &matrix, int32_t side_length, double cell_size = 0.1);
  ~SquareGridBase();

  double GetCellSize() const { return cell_size_; }

  inline GridCoordinate GetCoordinateFromID(int64_t id) { return IDToCoordinate(id); }
  inline int64_t GetIDFromCoordinate(int32_t x, int32_t y) { return CoordinateToID(x, y); }

  SquareCellBase<T> *GetCell(int64_t id);
  SquareCellBase<T> *GetCell(int32_t x, int32_t y);

  std::vector<SquareCellBase<T> *> GetNeighbours(int32_t x, int32_t y, bool allow_diag);
  std::vector<SquareCellBase<T> *> GetNeighbours(int64_t id, bool allow_diag = true);

  inline void SetCellLabel(int32_t x, int32_t y, SquareCellLabel label)
  {
    GridBase<SquareCellBase<T> *>::GetTileAtGridCoordinate(x, y)->label = label;
  }

  inline void SetCellLabel(int64_t id, SquareCellLabel label)
  {
    auto coordinate = IDToCoordinate(id);
    GridBase<SquareCellBase<T> *>::GetTileAtGridCoordinate(coordinate.GetX(), coordinate.GetY())->label = label;
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

using SquareGrid = SquareGridBase<double>;
}

#include "details/square_grid_base.hpp"

#endif /* SQUARE_GRID_HPP */
