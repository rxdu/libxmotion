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

SquareGrid::SquareGrid(int32_t row_num, int32_t col_num, double cell_size) : row_num_(row_num),
                                                                             col_num_(col_num),
                                                                             cell_size_(cell_size)
{
    assert((row_num > 0 && col_num > 0));

    ResizeGrid(col_num, row_num);

    for (int32_t y = 0; y < row_num; y++)
        for (int32_t x = 0; x < col_num; x++)
        {
            SquareCell *new_cell = new SquareCell(y, x, CoordinateToID(y, x));
            new_cell->UpdateGeometry(cell_size);
            SetTileAtRawCoordinate(x, y, new_cell);
        }
}

SquareGrid::~SquareGrid()
{
    for (int32_t y = 0; y < row_num_; y++)
        for (int32_t x = 0; x < col_num_; x++)
        {
            delete GetTileAtRawCoordinate(x, y);
        }
}

int64_t SquareGrid::GetIDFromCoordinate(int32_t x_col, int32_t y_row)
{
    return CoordinateToID(y_row, x_col);
}

GridCoordinate SquareGrid::GetCoordinateFromID(int64_t id)
{
    return IDToCoordinate(id);
}

void SquareGrid::SetCellLabel(int32_t x_col, int32_t y_row, SquareCellLabel label)
{
    GetTileAtGridCoordinate(x_col, y_row)->label = label;
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

SquareCell *SquareGrid::GetCell(int32_t x_col, int32_t y_row)
{
    return GetTileAtGridCoordinate(x_col, y_row);
}

std::vector<SquareCell *> SquareGrid::GetNeighbours(int32_t x_col, int32_t y_row, bool allow_diag)
{
    std::vector<GridCoordinate> candidates;

    if (allow_diag)
    {
        for (int32_t x = x_col - 1; x <= x_col + 1; ++x)
            for (int32_t y = y_row - 1; y <= y_row + 1; ++y)
            {
                if (x == x_col && y == y_row)
                    continue;
                candidates.emplace_back(x, y);
            }
    }
    else
    {
        std::vector<GridCoordinate> candidates;
        candidates.emplace_back(x_col, y_row + 1);
        candidates.emplace_back(x_col, y_row - 1);
        candidates.emplace_back(x_col + 1, y_row);
        candidates.emplace_back(x_col - 1, y_row);
    }

    std::vector<SquareCell *> neighbours;
    for (auto &can : candidates)
    {
        int32_t x = can.GetX();
        int32_t y = can.GetY();
        if (x >= 0 && x < col_num_ &&
            y >= 0 && y < row_num_)
            neighbours.push_back(GetTileAtRawCoordinate(x, y));
    }

    return neighbours;
}

std::vector<SquareCell *> SquareGrid::GetNeighbours(int64_t id, bool allow_diag)
{
    auto coordinate = IDToCoordinate(id);
    return GetNeighbours(coordinate.GetX(), coordinate.GetY(), allow_diag);
}