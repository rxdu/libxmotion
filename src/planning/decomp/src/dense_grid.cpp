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

void DenseGrid::SetupGridWithMatrix(const Eigen::MatrixXd &mat)
{
    ResizeGrid(mat.cols(), mat.rows());
    grid_tiles_ = mat;
}

void DenseGrid::SetValueAtCoordinate(int64_t x, int64_t y, double val)
{
    SetTileAtGridCoordinate(x, y, val);
}

double DenseGrid::GetValueAtCoordinate(int64_t x, int64_t y)
{
    return GetTileAtGridCoordinate(x, y);
}

void DenseGrid::AddGrid(const DenseGrid &other)
{
    assert(this->SizeX() == other.SizeX() && this->SizeY() == other.SizeY());

    this->grid_tiles_ += other.grid_tiles_;
}

void DenseGrid::SubtractGrid(const DenseGrid &other)
{
    assert(this->SizeX() == other.SizeX() && this->SizeY() == other.SizeY());

    this->grid_tiles_ -= other.grid_tiles_;
}

void DenseGrid::AddGrid(DenseGrid *other)
{
    assert(this->SizeX() == other->SizeX() && this->SizeY() == other->SizeY());

    this->grid_tiles_ += other->grid_tiles_;
}

void DenseGrid::SubtractGrid(DenseGrid *other)
{
    assert(this->SizeX() == other->SizeX() && this->SizeY() == other->SizeY());

    this->grid_tiles_ -= other->grid_tiles_;
}

Eigen::MatrixXd DenseGrid::GetGridMatrix(bool normalize)
{
    Eigen::MatrixXd grid_matrix = grid_tiles_;
    if (normalize)
        grid_matrix = (grid_tiles_ + Eigen::MatrixXd::Ones(grid_tiles_.rows(), grid_tiles_.cols()) * grid_tiles_.minCoeff()) / (grid_tiles_.maxCoeff() - grid_tiles_.minCoeff()) * 1.0;

    return grid_matrix;
}

// threshold: 0 - 1
std::shared_ptr<SquareGrid> DenseGrid::ConvertToSquareGrid(int32_t side_length, double threshold)
{
    Eigen::MatrixXd matrix = GetGridMatrix(true);

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

    Eigen::MatrixXd occupancy_matrix;
    if (shrink_x || shrink_y)
    {
        occupancy_matrix = Eigen::MatrixXd::Ones(grid_size_y * side_length, grid_size_x * side_length);

        int32_t x_start = 0, y_start = 0;
        if (shrink_x)
            x_start = center_x % side_length;
        if (shrink_y)
            y_start = center_y % side_length;

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
                    if (occupancy_matrix(j, i) > threshold)
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