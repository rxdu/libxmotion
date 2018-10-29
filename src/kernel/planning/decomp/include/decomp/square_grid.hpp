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

#include "decomp/details/rect_grid_base.hpp"

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

////////////////////////////////////////////////////////////////////

template <typename AttributeType>
struct SquareCellBase
{
  struct GridPoint
  {
    GridPoint(double xval = 0, double yval = 0) : x(xval), y(yval){};

    double x;
    double y;
  };

  SquareCellBase() = default;
  SquareCellBase(int32_t xval, int32_t yval, int64_t idval = -1) : x(xval),
                                                                   y(yval),
                                                                   id(idval) {}

  ~SquareCellBase() = default;
  SquareCellBase(const SquareCellBase &other) = default;
  SquareCellBase &operator=(const SquareCellBase &other) = default;
  SquareCellBase(SquareCellBase &&other) = default;
  SquareCellBase &operator=(SquareCellBase &&other) = default;

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

  // define extra attributes if the default ones are not enough
  AttributeType extra_attribute;

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

  inline void PrintInfo() const
  {
    std::cout << "cell " << id << " : " << x << " , " << y
              << " ; center : " << center.x << " , " << center.y << std::endl;
  }
};

using SquareCell = SquareCellBase<double>;

////////////////////////////////////////////////////////////////////

template <typename T>
class SquareGridBase : public RectGridBase<SquareCellBase<T> *>
{
public:
  SquareGridBase(int32_t size_x, int32_t size_y, double cell_size = 0.1);
  SquareGridBase(const Eigen::MatrixXd &matrix, int32_t side_length, double cell_size = 0.1);
  ~SquareGridBase();

  SquareGridBase(const SquareGridBase<T> &other);
  SquareGridBase<T> &operator=(const SquareGridBase<T> &other);
  SquareGridBase(SquareGridBase<T> &&other);
  SquareGridBase<T> &operator=(SquareGridBase<T> &&other);

  double GetCellSize() const { return cell_size_; }

  inline RectGridIndex GetIndexFromID(int64_t id) { return IDToIndex(id); }
  inline int64_t GetIDFromIndex(int32_t x, int32_t y) { return IndexToID(x, y); }

  SquareCellBase<T> *GetCell(int64_t id);
  SquareCellBase<T> *GetCell(int32_t x, int32_t y);

  std::vector<SquareCellBase<T> *> GetNeighbours(int32_t x, int32_t y, bool allow_diag);
  std::vector<SquareCellBase<T> *> GetNeighbours(int64_t id, bool allow_diag = true);

  inline void SetCellLabel(int32_t x, int32_t y, SquareCellLabel label)
  {
    RectGridBase<SquareCellBase<T> *>::GetTileAtGridCoordinate(x, y)->label = label;
  }

  inline void SetCellLabel(int64_t id, SquareCellLabel label)
  {
    auto coordinate = IDToIndex(id);
    RectGridBase<SquareCellBase<T> *>::GetTileAtGridCoordinate(coordinate.GetX(), coordinate.GetY())->label = label;
  }

private:
  double cell_size_;

  // IndexToID() and IDToIndex() are the only places that define
  //  the mapping between index and id
  inline int64_t IndexToID(int32_t x, int32_t y)
  {
    return y * RectGridBase<SquareCellBase<T> *>::size_x_ + x;
  }

  inline RectGridIndex IDToIndex(int64_t id)
  {
    return RectGridIndex(id % RectGridBase<SquareCellBase<T> *>::size_x_, id / RectGridBase<SquareCellBase<T> *>::size_x_);
  }
};

using SquareGrid = SquareGridBase<double>;
} // namespace librav

#include "details/square_grid_impl.hpp"

#endif /* SQUARE_GRID_HPP */
