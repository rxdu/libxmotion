/* 
 * grid_decomposer.cpp
 * 
 * Created on: Apr 05, 2018 12:00
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "decomp/grid_decomposer.hpp"

#include <vector>
#include <iostream>

// #define DEBUG_PRINTF

using namespace librav;

std::shared_ptr<SquareGrid> GridDecomposer::CreateSquareGridFromDenseGrid(std::shared_ptr<DenseGrid> grid, int32_t side_length)
{
    return CreateSquareGridFromMatrix(grid->GetGridMatrix(false), side_length);
}

std::shared_ptr<SquareGrid> GridDecomposer::CreateSquareGridFromMatrix(const Eigen::MatrixXd &matrix, int32_t side_length)
{
    // determine size of grid
    int32_t center_x = matrix.cols() / 2;
    int32_t center_y = matrix.rows() / 2;

    int32_t grid_size_x, grid_size_y;
    bool shrink_x = false;
    bool shrink_y = false;
    if (matrix.cols() % side_length != 0)
        shrink_x = true;
    if (matrix.rows() % side_length != 0)
        shrink_y = true;
    grid_size_x = (center_x / side_length) * 2;
    grid_size_y = (center_y / side_length) * 2;

#ifdef DEBUG_PRINTF
    std::cout << "matrix: " << matrix.cols() << " , " << matrix.rows() << std::endl;
    std::cout << "center: " << center_x << " , " << center_y << std::endl;
    std::cout << "grid size: " << grid_size_x << " , " << grid_size_y << std::endl;
#endif

    Eigen::MatrixXd occupancy_matrix;
    if (shrink_x || shrink_y)
    {
        occupancy_matrix = Eigen::MatrixXd::Ones(grid_size_y * side_length, grid_size_x * side_length);
#ifdef DEBUG_PRINTF
        std::cout << "occupancy_matrix size: " << grid_size_x * side_length << " , " << grid_size_y * side_length << std::endl;
#endif
        int32_t x_start = 0, y_start = 0;
        if (shrink_x)
            x_start = center_x % side_length;
        if (shrink_y)
            y_start = center_y % side_length;
#ifdef DEBUG_PRINTF
            std::cout << "index start: " << x_start << " , " << y_start << std::endl;
#endif
        occupancy_matrix = matrix.block(y_start, x_start, occupancy_matrix.rows(), occupancy_matrix.cols());
    }
    else
    {
        occupancy_matrix = matrix;
    }

    // create new grid
    auto grid = std::make_shared<SquareGrid>(grid_size_x, grid_size_y);

    // determine occupancy of grid
    int32_t half_size_x = grid_size_x / 2;
    int32_t half_size_y = grid_size_y / 2;
    for (int64_t x = 0; x < grid->SizeX(); ++x)
        for (int64_t y = 0; y < grid->SizeY(); ++y)
        {
            bool occupied = false;
            int32_t xmin = x * side_length;
            int32_t xmax = xmin + side_length;
            int32_t ymin = y * side_length;
            int32_t ymax = ymin + side_length;
            for (int i = xmin; i < xmax; ++i)
            {
                for (int j = ymin; j < ymax; ++j)
                {
                    if (occupancy_matrix(j, i) != 0)
                    {
                        grid->SetCellLabel(x, y, SquareCellLabel::OCCUPIED);
                        occupied = true;
                        break;
                    }
                }
                if (occupied)
                    break;
            }
        }

    return grid;
}
