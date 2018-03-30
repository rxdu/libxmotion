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
    SetTileAtGridCoordinate(x, y, val);
}

double DenseGrid::GetValueAtCoordinate(int64_t x, int64_t y)
{
    return GetTileAtGridCoordinate(x, y);
}

Eigen::MatrixXd DenseGrid::GetGridMatrix(bool normalize)
{
    Eigen::MatrixXd grid_matrix = grid_tiles_;
    if (normalize)
        grid_matrix = grid_matrix / grid_matrix.maxCoeff() * 1.0;
    return grid_matrix;
}