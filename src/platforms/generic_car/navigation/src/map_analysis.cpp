/* 
 * map_analysis.cpp
 * 
 * Created on: Apr 05, 2018 18:23
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include <iostream>

#include "navigation/map_analysis.hpp"
#include "graph/algorithms/dijkstra.hpp"

using namespace librav;

void MapAnalysis::GenerateGraphCostMap(RoadSquareGrid *grid, Graph_t<RoadSquareCell *> *graph)
{
    Eigen::MatrixXd cost_matrix;
    cost_matrix = Eigen::MatrixXd::Zero(grid->SizeY(), grid->SizeX());

    std::vector<double> cost;
    for (int32_t i = 0; i < grid->SizeX(); ++i)
        for (int32_t j = 0; j < grid->SizeY(); ++j)
        {
            auto vtx = graph->FindVertex(grid->GetCell(i, j)->GetUniqueID());
            if (vtx != graph->vertex_end())
                cost_matrix(j, i) = vtx->g_astar_;
        }

    cost_matrix = (cost_matrix + Eigen::MatrixXd::Ones(cost_matrix.rows(), cost_matrix.cols()) * cost_matrix.minCoeff()) / (cost_matrix.maxCoeff() - cost_matrix.minCoeff()) * 1.0;

    for (int32_t i = 0; i < grid->SizeX(); ++i)
        for (int32_t j = 0; j < grid->SizeY(); ++j)
            grid->GetCell(i, j)->cost_map = cost_matrix(j, i);
}

void MapAnalysis::GenerateGraphCostMap(RoadSquareGrid *grid, std::string channel)
{
    Eigen::MatrixXd cost_matrix;
    cost_matrix = Eigen::MatrixXd::Zero(grid->SizeY(), grid->SizeX());

    std::vector<double> cost;
    for (int32_t i = 0; i < grid->SizeX(); ++i)
        for (int32_t j = 0; j < grid->SizeY(); ++j)
        {
            auto cell = grid->GetCell(i, j);
            if (cell->extra_attribute.cost_.find(channel) != cell->extra_attribute.cost_.end())
                cost_matrix(j, i) = cell->extra_attribute.cost_[channel];
            else
                cost_matrix(j, i) = 0;
        }

    cost_matrix = (cost_matrix + Eigen::MatrixXd::Ones(cost_matrix.rows(), cost_matrix.cols()) * cost_matrix.minCoeff()) / (cost_matrix.maxCoeff() - cost_matrix.minCoeff()) * 1.0;

    for (int32_t i = 0; i < grid->SizeX(); ++i)
        for (int32_t j = 0; j < grid->SizeY(); ++j)
            grid->GetCell(i, j)->cost_map = cost_matrix(j, i);
}

void MapAnalysis::GenerateTrafficDensityCostMap(RoadSquareGrid *grid)
{
    Eigen::MatrixXd cost_matrix;
    cost_matrix = Eigen::MatrixXd::Zero(grid->SizeY(), grid->SizeX());

    std::vector<double> cost;
    for (int32_t i = 0; i < grid->SizeX(); ++i)
        for (int32_t j = 0; j < grid->SizeY(); ++j)
        {
            auto cell = grid->GetCell(i, j);
            cost_matrix(j, i) = cell->extra_attribute.cost_.size();
        }

    cost_matrix = (cost_matrix + Eigen::MatrixXd::Ones(cost_matrix.rows(), cost_matrix.cols()) * cost_matrix.minCoeff()) / (cost_matrix.maxCoeff() - cost_matrix.minCoeff()) * 1.0;

    for (int32_t i = 0; i < grid->SizeX(); ++i)
        for (int32_t j = 0; j < grid->SizeY(); ++j)
            grid->GetCell(i, j)->cost_map = cost_matrix(j, i);
}