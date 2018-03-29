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

#include "decomp/grid_base.hpp"

namespace librav
{
/*
 * Coordinate System:
 *
 *		   <<		   column       >>
 *  	   o --------------------> x
 *	^	   |
 *	^	   |
 *		   |
 * row	 |
 *		   |
 *	v	   |
 *	v	   v
 *       y
 */

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

struct SquareCell
{
  SquareCell(int32_t r, int32_t c, int64_t id_val = -1) : row(r),
                                                          col(c),
                                                          id(id_val) {}

  // for easy reference, maybe unnecessary for some applications
  int64_t id;

  // topological attributes
  int32_t row;
  int32_t col;
  SquareCellLabel label = SquareCellLabel::FREE;

  // geometrical attributes
  // 4 vertices in the order:
  // 0 - top left, 1 - top right
  // 2 - bottom left, 3 - bottom right
  GridPoint vertices[4];
  GridPoint center;

  void UpdateGeometry(double size)
  {
    vertices[0].x = size * col;
    vertices[0].y = size * row;

    vertices[1].x = size * (col + 1);
    vertices[1].y = size * row;

    vertices[2].x = size * col;
    vertices[2].y = size * (row + 1);

    vertices[3].x = size * (col + 1);
    vertices[3].y = size * (row + 1);

    center.x = vertices[0].x + size / 2.0;
    center.y = vertices[0].y + size / 2.0;
  }
};

class SquareGrid : public GridBase<SquareCell *>
{
public:
  SquareGrid(int32_t row_num, int32_t col_num, double cell_size = 0.1);
  ~SquareGrid();

  int32_t row_num_;
  int32_t col_num_;
  double cell_size_;

public:
  void SetCellLabel(int32_t x_col, int32_t y_row, SquareCellLabel label);
  void SetCellLabel(int64_t id, SquareCellLabel label);

  GridCoordinate GetCoordinateFromID(int64_t id);
  int64_t GetIDFromCoordinate(int32_t x_col, int32_t y_row);

  SquareCell *GetCell(int64_t id);
  SquareCell *GetCell(int32_t x_col, int32_t y_row);
  std::vector<SquareCell *> GetNeighbours(int32_t x_col, int32_t y_row, bool allow_diag = true);
  std::vector<SquareCell *> GetNeighbours(int64_t id, bool allow_diag = true);

private:
  // CoordinateToID() and IDToCoordinate() are the only places that define
  //  the mapping between coordinate and id
  inline int64_t CoordinateToID(int32_t row, int32_t col)
  {
    return row * col_num_ + col;
  }

  inline GridCoordinate IDToCoordinate(int64_t id)
  {
    return GridCoordinate(id % col_num_, id / col_num_);
  }
};
}

#endif /* SQUARE_GRID_HPP */
