/* 
 * road_map_analysis.cpp
 * 
 * Created on: Apr 05, 2018 18:23
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "navigation/road_map_analysis.hpp"

using namespace librav;

// #define ALLOW_DIAGONAL_MOVE

GetSquareGridNeighbour::GetSquareGridNeighbour(std::shared_ptr<SquareGrid> sg) : sgrid_(sg)
{
}

std::vector<std::tuple<SquareCell *, double>> GetSquareGridNeighbour::operator()(SquareCell *cell)
{
    std::vector<std::tuple<SquareCell*, double>> adjacent_cells;

#ifdef ALLOW_DIAGONAL_MOVE
    auto neighbours = sgrid_->GetNeighbours(cell->id, true);
#else
    auto neighbours = sgrid_->GetNeighbours(cell->id, false);
#endif

    for (auto nb : neighbours)
    {
        double x_err = cell->center.x - nb->center.x;
        double y_err = cell->center.y - nb->center.y;
        double dist = std::sqrt(x_err * x_err + y_err * y_err);
        adjacent_cells.push_back(std::make_tuple(nb, dist));
    }

    return adjacent_cells;
}
