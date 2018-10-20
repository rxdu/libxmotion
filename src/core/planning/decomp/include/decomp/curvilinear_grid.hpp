/* 
 * curvilinear_grid.hpp
 * 
 * Created on: Oct 20, 2018 09:13
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef CURVILINEAR_GRID_HPP
#define CURVILINEAR_GRID_HPP

#include <cstdint>
#include <vector>

#include "geometry/parametric_curve.hpp"

namespace librav
{
/*
 * Coordinate System:
 * 
 *               s
 *               ^
 *               |
 *		          |
 *		         |
 *		        |     parametric curve
 *		       |
 *		      |
 *		      |
 *	 <----- o ----- delta
 */

////////////////////////////////////////////////////////////////////

// x - longitudinal/tangential , y - lateral/normal
class CurviGridIndex
{
public:
  CurviGridIndex() : coordinate_x_(0), coordinate_y_(0) {}
  CurviGridIndex(int64_t x = 0, int64_t y = 0) : coordinate_x_(x), coordinate_y_(y) {}
  ~CurviGridIndex() = default;

  inline int64_t GetX() const { return coordinate_x_; };
  inline int64_t GetY() const { return coordinate_y_; };
  inline void SetX(int64_t x) { coordinate_x_ = x; };
  inline void SetY(int64_t y) { coordinate_y_ = y; };
  inline void SetXY(int64_t x, int64_t y)
  {
    coordinate_x_ = x;
    coordinate_y_ = y;
  };

private:
  int64_t coordinate_x_;
  int64_t coordinate_y_;
};

////////////////////////////////////////////////////////////////////

template <typename AttributeType>
struct CurvilinearCellBase
{
  struct GridPoint
  {
    GridPoint(double _s = 0, double _d = 0) : s(_s), delta(_d){};

    double s;
    double delta;
  };

  CurvilinearCellBase(int32_t xval, int32_t yval, int64_t idval = -1) : index(CurviGridIndex(xval, yval)),
                                                                        id(idval) {}
  ~CurvilinearCellBase() = default;

  // for easy reference, maybe unnecessary for some applications
  int64_t id = -1;

  // topological attributes
  CurviGridIndex index;

  // geometrical attributes
  // 4 vertices in the order:
  // 0 - top left, 1 - top right
  // 2 - bottom left, 3 - bottom right
  GridPoint vertices[4];
  GridPoint center;

  // other application specific attributes
  // you can assign values to "cost_map" for visualization
  double cost_map = 0.0;

  // define extra attributes if the default ones are not enough
  AttributeType extra_attribute;

  inline int64_t GetUniqueID() const
  {
    return id;
  }

  inline void UpdateGeometry(double size)
  {
    // vertices[0].x = size * x;
    // vertices[0].y = size * y;

    // vertices[1].x = size * (x + 1);
    // vertices[1].y = size * y;

    // vertices[2].x = size * x;
    // vertices[2].y = size * (y + 1);

    // vertices[3].x = size * (x + 1);
    // vertices[3].y = size * (y + 1);

    // center.x = vertices[0].x + size / 2.0;
    // center.y = vertices[0].y + size / 2.0;
  }

  inline void Print() const
  {
    // std::cout << "cell " << id << " : " << x << " , " << index
    //           << " ; center : " << center.x << " , " << center.y << std::endl;
  }
};

using CurvilinearCell = CurvilinearCellBase<double>;

////////////////////////////////////////////////////////////////////

template <typename T>
class CurvilinearGridBase
{
public:
  CurvilinearGridBase(ParametricCurve pcurve, double s_step, double d_step);
  ~CurvilinearGridBase() = default;

  using CellType = CurvilinearCellBase<T>;

  // double GetCellSize() const { return cell_size_; }

  // inline CurviGridIndex GetCoordinateFromID(int64_t id) { return IDToCoordinate(id); }
  // inline int64_t GetIDFromCoordinate(int32_t x, int32_t y) { return CoordinateToID(x, y); }

  // CurvilinearCellBase<T> *GetCell(int64_t id);
  // CurvilinearCellBase<T> *GetCell(int32_t x, int32_t y);

  // std::vector<SquareCellBase<T> *> GetNeighbours(int32_t x, int32_t y, bool allow_diag);
  // std::vector<SquareCellBase<T> *> GetNeighbours(int64_t id, bool allow_diag = true);

  // inline void SetCellLabel(int32_t x, int32_t y, SquareCellLabel label)
  // {
  //   GridBase<SquareCellBase<T> *>::GetTileAtGridCoordinate(x, y)->label = label;
  // }

  // inline void SetCellLabel(int64_t id, SquareCellLabel label)
  // {
  //   auto coordinate = IDToCoordinate(id);
  //   GridBase<SquareCellBase<T> *>::GetTileAtGridCoordinate(coordinate.GetX(), coordinate.GetY())->label = label;
  // }

private:
  ParametricCurve curve_;

  double s_step_;
  double delta_step_;

  std::vector<std::vector<CellType>> grid_tiles_;

  // CoordinateToID() and IDToCoordinate() are the only places that define
  //  the mapping between coordinate and id
  // inline int64_t CoordinateToID(int32_t x, int32_t y)
  // {
  //   return y * GridBase<SquareCellBase<T> *>::size_x_ + x;
  // }

  // inline CurviGridIndex IDToCoordinate(int64_t id)
  // {
  //   return CurviGridIndex(id % GridBase<SquareCellBase<T> *>::size_x_, id / GridBase<SquareCellBase<T> *>::size_x_);
  // }
};

using CurvilinearGrid = CurvilinearGridBase<double>;
} // namespace librav

#include "details/curvilinear_grid_impl.hpp"

#endif /* CURVILINEAR_GRID_HPP */
