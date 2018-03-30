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

SquareGrid::~SquareGrid()
{
    for (int32_t y = 0; y < size_y_; y++)
        for (int32_t x = 0; x < size_x_; x++)
        {
            delete GetTileAtRawCoordinate(x, y);
        }
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
            for (int32_t yi = y - 1; yi <= yi + 1; ++yi)
            {
                if (xi == x && yi == y)
                    continue;
                candidates.emplace_back(xi, yi);
            }
    }
    else
    {
        std::vector<GridCoordinate> candidates;
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
        if (xi >= 0 && xi < size_x_ &&
            yi >= 0 && yi < size_y_)
            neighbours.push_back(GetTileAtRawCoordinate(xi, yi));
    }

    return neighbours;
}

std::vector<SquareCell *> SquareGrid::GetNeighbours(int64_t id, bool allow_diag)
{
    auto coordinate = IDToCoordinate(id);
    return GetNeighbours(coordinate.GetX(), coordinate.GetY(), allow_diag);
}