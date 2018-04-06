/* 
 * road_map_analysis.cpp
 * 
 * Created on: Apr 05, 2018 18:23
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include <iostream>

#include "navigation/road_map_analysis.hpp"

using namespace librav;

#define ALLOW_DIAGONAL_MOVE

GetSquareGridNeighbour::GetSquareGridNeighbour(std::shared_ptr<SquareGrid> sg) : sgrid_(sg)
{
}

std::vector<std::tuple<SquareCell *, double>> GetSquareGridNeighbour::operator()(SquareCell *cell)
{
    std::vector<std::tuple<SquareCell *, double>> adjacent_cells;

    if (cell->label != SquareCellLabel::FREE)
        return adjacent_cells;

#ifdef ALLOW_DIAGONAL_MOVE
    auto neighbours = sgrid_->GetNeighbours(cell->id, true);
#else
    auto neighbours = sgrid_->GetNeighbours(cell->id, false);
#endif

    for (auto nb : neighbours)
    {
        if (nb->label == SquareCellLabel::FREE)
        {
            double x_err = cell->center.x - nb->center.x;
            double y_err = cell->center.y - nb->center.y;
            double dist = std::sqrt(x_err * x_err + y_err * y_err);
            adjacent_cells.push_back(std::make_tuple(nb, dist));
        }
    }

    return adjacent_cells;
}

void RoadMapAnalysis::GenerateGraphCostMap(SquareGrid *grid, Graph_t<SquareCell *> *graph)
{
    Eigen::MatrixXd cost_matrix;
    cost_matrix = Eigen::MatrixXd::Zero(grid->SizeY(), grid->SizeX());

    std::vector<double> cost;
    for (int32_t i = 0; i < grid->SizeX(); ++i)
        for (int32_t j = 0; j < grid->SizeY(); ++j)
        {
            int64_t id = grid->GetCell(i, j)->GetUniqueID();
            if (graph->FindVertex(id) != graph->vertex_end())
                cost_matrix(j, i) = vtx->g_astar_;
        }

    cost_matrix = (cost_matrix + Eigen::MatrixXd::Ones(cost_matrix.rows(), cost_matrix.cols()) * cost_matrix.minCoeff()) / (cost_matrix.maxCoeff() - cost_matrix.minCoeff()) * 1.0;

    for (int32_t i = 0; i < grid->SizeX(); ++i)
        for (int32_t j = 0; j < grid->SizeY(); ++j)
            grid->GetCell(i, j)->cost_map = cost_matrix(j, i);
}