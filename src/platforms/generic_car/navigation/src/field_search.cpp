/* 
 * field_search.cpp
 * 
 * Created on: Mar 08, 2018 17:36
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "navigation/field_search.hpp"

using namespace librav;

// #define ALLOW_DIAGONAL_MOVE

GetFieldTileNeighbour::GetFieldTileNeighbour(std::shared_ptr<CollisionField> cf) : field_(cf)
{
}

std::vector<std::tuple<FieldTile, double>> GetFieldTileNeighbour::operator()(FieldTile tile)
{
    std::vector<std::tuple<FieldTile, double>> adjacent_tiles;

#ifdef ALLOW_DIAGONAL_MOVE
    for (int32_t j = tile.position_y_ - 1; j <= tile.position_y_ + 1; ++j)
        for (int32_t i = tile.position_x_ - 1; i <= tile.position_x_ + 1; ++i)
        {
            if (i == tile.position_x_ && j == tile.position_y_)
                continue;

            if (j >= 0 && j < field_->SizeY() &&
                i >= 0 && i < field_->SizeX())
            {
                FieldTile ntile(i, j, field_);
                int32_t x_err = ntile.position_x_ - tile.position_x_;
                int32_t y_err = ntile.position_y_ - tile.position_y_;

                double dist = std::sqrt(x_err * x_err + y_err * y_err);
                double edge_cost = ntile.GetCollisionThreat() - tile.GetCollisionThreat() + dist * 0.001;
                adjacent_tiles.emplace_back(ntile, edge_cost);
            }
        }
#else
    int32_t pos_x[4];
    int32_t pos_y[4];

    pos_x[0] = tile.position_x_ - 1;
    pos_y[0] = tile.position_y_;

    pos_x[1] = tile.position_x_ + 1;
    pos_y[1] = tile.position_y_;

    pos_x[2] = tile.position_x_;
    pos_y[2] = tile.position_y_ - 1;

    pos_x[3] = tile.position_x_;
    pos_y[3] = tile.position_y_ + 1;

    for (int i = 0; i < 4; ++i)
    {
        if (pos_y[i] >= 0 && pos_y[i] < field_->SizeY() && pos_x[i] >= 0 && pos_x[i] < field_->SizeX())
        {
            FieldTile ntile(pos_x[i], pos_y[i], field_);
            double edge_cost = ntile.GetCollisionThreat() - tile.GetCollisionThreat();
            adjacent_tiles.emplace_back(ntile, edge_cost);
        }
    }
#endif

    return adjacent_tiles;
}

Eigen::MatrixXi FieldSearch::GetPathWaypoints(const std::vector<FieldTile> &path)
{
    Eigen::MatrixXi path_matrix;

    path_matrix.resize(path.size(), 2);
    int32_t path_idx = 0;
    for (const auto &wp : path)
    {
        path_matrix(path_idx, 0) = wp.position_x_;
        path_matrix(path_idx, 1) = wp.position_y_;
        ++path_idx;
    }

    return path_matrix;
}