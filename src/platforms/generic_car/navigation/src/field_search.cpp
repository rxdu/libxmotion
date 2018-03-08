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

GetFieldTileNeighbour::GetFieldTileNeighbour(std::shared_ptr<CollisionField> cf) : field_(cf)
{
}

std::vector<std::tuple<FieldTile, double>> GetFieldTileNeighbour::operator()(FieldTile tile)
{
    std::vector<std::tuple<FieldTile, double>> adjacent_tiles;

    for (int32_t j = tile.position_y_ - 1; j <= tile.position_y_ + 1; ++j)
        for (int32_t i = tile.position_x_ - 1; i <= tile.position_x_ + 1; ++i)
        {
            if (i == tile.position_x_ && j == tile.position_y_)
                continue;

            if (j >= 0 && j < field_->SizeY() &&
                i >= 0 && i < field_->SizeX())
            {
                FieldTile ntile(i, j, field_);
                double edge_cost = ntile.GetCollisionThreat() - tile.GetCollisionThreat();
                // std::cout << "edge cost: " << edge_cost << std::endl;
                adjacent_tiles.emplace_back(ntile, edge_cost);
            }
        }

    return adjacent_tiles;
}