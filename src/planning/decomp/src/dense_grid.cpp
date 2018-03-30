/* 
 * dense_grid.cpp
 * 
 * Created on: Mar 30, 2018 11:04
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "decomp/dense_grid.hpp"

using namespace librav;

DenseGrid::DenseGrid(int32_t size_x, int32_t size_y) : GridBase<double>(size_x, size_y)
{
}

void DenseGrid::SetValueAtCoordinate(int64_t x, int64_t y, double val)
{
}

double DenseGrid::GetValueAtCoordinate(int64_t x, int64_t y)
{
}