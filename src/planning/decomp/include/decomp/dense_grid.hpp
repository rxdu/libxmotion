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
class DenseGrid : public GridBase<double>
{
  public:
    DenseGrid(int32_t size_x, int32_t size_y);
    ~DenseGrid();
};
}

#endif /* DENSE_GRID_HPP */
