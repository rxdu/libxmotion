/* 
 * square_grid.cpp
 * 
 * Created on: Mar 28, 2018 23:07
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "decomp/square_grid.hpp"

using namespace librav;

SquareGrid::SquareGrid(int32_t size_x, int32_t size_y, double cell_size) : GridBase<SquareCell *>(size_x, size_y),
                                                                           cell_size_(cell_size)
{
    assert((size_x > 0 && size_y > 0));

    for (int32_t y = 0; y < size_y; y++)
        for (int32_t x = 0; x < size_x; x++)
        {
            SquareCell *new_cell = new SquareCell(x, y, CoordinateToID(x, y));
            new_cell->UpdateGeometry(cell_size);
            SetTileAtRawCoordinate(x, y, new_cell);
        }
}

SquareGrid::SquareGrid(const Eigen::MatrixXd &matrix, int32_t side_length, double cell_size) : GridBase<SquareCell *>(0, 0),
                                                                                               cell_size_(cell_size)
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
    this->ResizeGrid(grid_size_x, grid_size_y);
    for (int32_t y = 0; y < grid_size_y; y++)
        for (int32_t x = 0; x < grid_size_x; x++)
        {
            SquareCell *new_cell = new SquareCell(x, y, CoordinateToID(x, y));
            new_cell->UpdateGeometry(cell_size);
            SetTileAtRawCoordinate(x, y, new_cell);
        }

    // determine occupancy of grid
    int32_t half_size_x = grid_size_x / 2;
    int32_t half_size_y = grid_size_y / 2;
    for (int64_t x = 0; x < this->SizeX(); ++x)
        for (int64_t y = 0; y < this->SizeY(); ++y)
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
                        this->SetCellLabel(x, y, SquareCellLabel::OCCUPIED);
                        occupied = true;
                        break;
                    }
                }
                if (occupied)
                    break;
            }
        }
}

SquareGrid::~SquareGrid()
{
    for (int32_t y = 0; y < size_y_; y++)
        for (int32_t x = 0; x < size_x_; x++)
            delete GetTileAtRawCoordinate(x, y);
}

int64_t SquareGrid::GetIDFromCoordinate(int32_t x, int32_t y)
{
    return CoordinateToID(x, y);
}

GridCoordinate SquareGrid::GetCoordinateFromID(int64_t id)
{
    return IDToCoordinate(id);
}

void SquareGrid::SetCellLabel(int32_t x, int32_t y, SquareCellLabel label)
{
    GetTileAtGridCoordinate(x, y)->label = label;
}

void SquareGrid::SetCellLabel(int64_t id, SquareCellLabel label)
{
    auto coordinate = IDToCoordinate(id);
    GetTileAtGridCoordinate(coordinate.GetX(), coordinate.GetY())->label = label;
}

SquareCell *SquareGrid::GetCell(int64_t id)
{
    auto coordinate = IDToCoordinate(id);
    return GetTileAtGridCoordinate(coordinate.GetX(), coordinate.GetY());
}

SquareCell *SquareGrid::GetCell(int32_t x, int32_t y)
{
    return GetTileAtGridCoordinate(x, y);
}

std::vector<SquareCell *> SquareGrid::GetNeighbours(int32_t x, int32_t y, bool allow_diag)
{
    std::vector<GridCoordinate> candidates;
    if (allow_diag)
    {
        for (int32_t xi = x - 1; xi <= x + 1; ++xi)
            for (int32_t yi = y - 1; yi <= y + 1; ++yi)
            {
                if (xi == x && yi == y)
                    continue;
                candidates.emplace_back(xi, yi);
            }
    }
    else
    {
        candidates.emplace_back(x, y + 1);
        candidates.emplace_back(x, y - 1);
        candidates.emplace_back(x + 1, y);
        candidates.emplace_back(x - 1, y);
    }

    std::vector<SquareCell *> neighbours;
    for (auto &can : candidates)
    {
        int32_t xi = can.GetX();
        int32_t yi = can.GetY();
        if (xi >= 0 && xi < size_x_ && yi >= 0 && yi < size_y_)
            neighbours.push_back(GetTileAtRawCoordinate(xi, yi));
    }

    return neighbours;
}

std::vector<SquareCell *> SquareGrid::GetNeighbours(int64_t id, bool allow_diag)
{
    auto coordinate = IDToCoordinate(id);
    return GetNeighbours(coordinate.GetX(), coordinate.GetY(), allow_diag);
}