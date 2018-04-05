/* 
 * grid_decomposer.hpp
 * 
 * Created on: Apr 05, 2018 11:58
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef GRID_DECOMPOSER_HPP
#define GRID_DECOMPOSER_HPP

#include <memory>
#include "decomp/square_grid.hpp"
#include "decomp/dense_grid.hpp"

namespace librav
{
namespace GridDecomposer
{
std::shared_ptr<SquareGrid> CreateSquareGridFromDenseGrid(std::shared_ptr<DenseGrid> grid, int32_t side_length);

std::shared_ptr<SquareGrid> CreateSquareGridFromMatrix(const Eigen::MatrixXd& matrix, int32_t side_length);
};
}

#endif /* GRID_DECOMPOSER_HPP */
