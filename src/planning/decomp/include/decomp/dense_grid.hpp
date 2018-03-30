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

#include "decomp/grid_base.hpp"

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

class DenseGrid : public GridBase<double>
{
public:
  DenseGrid(int32_t size_x, int32_t size_y);
  ~DenseGrid() = default;

  void SetValueAtCoordinate(int64_t x, int64_t y, double val);
  double GetValueAtCoordinate(int64_t x, int64_t y);

  // public functions inherited from GridBase
  /****************************************************************
    void ResizeGrid(int64_t x, int64_t y);
    void SetOriginCoordinate(int64_t origin_x, int64_t origin_y);   
    *****************************************************************/

  Eigen::MatrixXd GetGridMatrix(bool normalize = false);
};
}

#endif /* DENSE_GRID_HPP */
