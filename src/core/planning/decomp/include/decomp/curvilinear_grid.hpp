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
#include <cassert>
#include <iostream>

#include <eigen3/Eigen/Dense>

#include "geometry/simple_point.hpp"
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
 *		        |   s-axis defined by a parametric curve
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
  CurviGridIndex() : index_x_(0), index_y_(0) {}
  CurviGridIndex(int64_t x = 0, int64_t y = 0) : index_x_(x), index_y_(y) {}
  ~CurviGridIndex() = default;

  CurviGridIndex(const CurviGridIndex &other) = default;
  CurviGridIndex(CurviGridIndex &&other) = default;
  CurviGridIndex &operator=(const CurviGridIndex &other) = default;
  CurviGridIndex &operator=(CurviGridIndex &&other) = default;

  inline int64_t GetX() const { return index_x_; };
  inline int64_t GetY() const { return index_y_; };
  inline void SetX(int64_t x) { index_x_ = x; };
  inline void SetY(int64_t y) { index_y_ = y; };
  inline void SetXY(int64_t x, int64_t y)
  {
    index_x_ = x;
    index_y_ = y;
  };

private:
  int64_t index_x_;
  int64_t index_y_;

  friend std::ostream &operator<<(std::ostream &os, const CurviGridIndex &idx)
  {
    os << "(x,y) : " << idx.index_x_ << " , " << idx.index_y_;
    return os;
  }
};

////////////////////////////////////////////////////////////////////

template <typename T>
struct CurvilinearCellBase
{
  struct GridPoint
  {
    GridPoint(double _s = 0, double _d = 0) : s(_s), delta(_d){};
    GridPoint(const GridPoint &other) = default;
    GridPoint(GridPoint &&other) = default;
    GridPoint &operator=(const GridPoint &other) = default;
    GridPoint &operator=(GridPoint &&other) = default;

    double s;
    double delta;

    SimplePoint position;
  };

  CurvilinearCellBase() = delete;
  CurvilinearCellBase(int32_t xval, int32_t yval, int64_t idval = -1) : index(CurviGridIndex(xval, yval)),
                                                                        id(idval) {}
  ~CurvilinearCellBase() = default;

  CurvilinearCellBase(const CurvilinearCellBase &other) = default;
  CurvilinearCellBase(CurvilinearCellBase &&other) = default;
  CurvilinearCellBase &operator=(const CurvilinearCellBase &other) = default;
  CurvilinearCellBase &operator=(CurvilinearCellBase &&other) = default;

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
  T extra_attribute;

  inline int64_t GetUniqueID() const
  {
    return id;
  }

  inline void UpdateGeometry(double s_step, double d_step, bool center_cell_null)
  {
    vertices[0].s = s_step * (index.GetX() + 1);
    vertices[1].s = s_step * (index.GetX() + 1);
    vertices[2].s = s_step * index.GetX();
    vertices[3].s = s_step * index.GetX();

    if (center_cell_null)
    {
      if (index.GetY() >= 0)
        vertices[0].delta = index.GetY() * d_step;
      else
        vertices[0].delta = (index.GetY() + 1) * d_step;
      vertices[1].delta = vertices[0].delta - d_step;
      vertices[2].delta = vertices[0].delta;
      vertices[3].delta = vertices[1].delta;
    }
    else
    {
      if (index.GetY() >= 0)
        vertices[0].delta = index.GetY() * d_step + d_step / 2.0;
      else
        vertices[0].delta = (index.GetY() + 1) * d_step - d_step / 2.0;
      vertices[1].delta = vertices[0].delta - d_step;
      vertices[2].delta = vertices[0].delta;
      vertices[3].delta = vertices[1].delta;
    }

    center.s = vertices[0].s - s_step / 2.0;
    center.delta = (vertices[0].delta + vertices[1].delta) / 2.0;
  }

  inline void PrintInfo() const
  {
    std::cout << "cell " << id << " ; index " << index
              << " ; center : " << center.s << " , " << center.delta << std::endl;
  }
};

using CurvilinearCell = CurvilinearCellBase<double>;

////////////////////////////////////////////////////////////////////

template <typename T>
class CurvilinearGridBase
{
public:
  CurvilinearGridBase(ParametricCurve pcurve, double s_step, double d_step, int32_t d_num);
  ~CurvilinearGridBase() = default;

  using CellType = CurvilinearCellBase<T>;
  using GridPoint = typename CurvilinearCellBase<T>::GridPoint;

  ParametricCurve curve_;
  std::vector<std::vector<CellType *>> grid_tiles_;

  /*--------------------------------------------------------------*/
  double GetCellSizeS() const { return s_step_; }
  double GetCellSizeDelta() const { return delta_step_; }

  int32_t GetTangentialGridNum() const { return grid_tiles_.size(); }
  int32_t GetNormalGridNum() const { return grid_tiles_.front().size(); }

  inline CurviGridIndex GetIndexFromID(int64_t id) { return IDToIndex(id); }
  inline int64_t GetIDFromIndex(int32_t x, int32_t y) { return IndexToID(x, y); }

  CurvilinearCellBase<T> *GetCell(int64_t id);
  CurvilinearCellBase<T> *GetCell(int32_t x, int32_t y);

  // std::vector<SquareCellBase<T> *> GetNeighbours(int32_t x, int32_t y, bool allow_diag);
  // std::vector<SquareCellBase<T> *> GetNeighbours(int64_t id, bool allow_diag = true);

  void PrintInfo()
  {
    std::cout << "------ Curvilinear Grid ------ " << std::endl;
    for (auto &row : grid_tiles_)
      for (auto &cell : row)
        cell->PrintInfo();
  }

private:
  double s_step_;
  double delta_step_;
  int32_t delta_num_;

  int32_t delta_half_num_;
  bool center_cell_null_;

  // local path coordinate
  // - s: starts from 0
  // - delta: positive - left, zero - on curve, negative -right
  SimplePoint ConvertToGlobalCoordinate(GridPoint pt);

  // IndexToID() and IDToIndex() are the only places that define
  //  the mapping between index and id
  int64_t IndexToID(int32_t x, int32_t y)
  {
    // id is offset by delta_half_num_ so that it starts from 0
    if (center_cell_null_ == 0)
      return x * delta_num_ + y + delta_half_num_;
    else
      return x * (delta_num_ + 1) + y + delta_half_num_;
  }

  inline CurviGridIndex IDToIndex(int64_t id)
  {
    int32_t idx_x, idx_y;

    if (center_cell_null_ == 0)
    {
      idx_x = id / delta_num_;
      idx_y = id % delta_num_ - delta_half_num_;
    }
    else
    {
      idx_x = id / (delta_num_ + 1);
      idx_y = id % (delta_num_ + 1) - delta_half_num_;
    }
    return CurviGridIndex(idx_x, idx_y);
  }
};

using CurvilinearGrid = CurvilinearGridBase<double>;
} // namespace librav

#include "details/curvilinear_grid_impl.hpp"

#endif /* CURVILINEAR_GRID_HPP */
