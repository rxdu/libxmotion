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

#define ALLOW_DIAGONAL_MOVE

GetSquareGridNeighbour::GetSquareGridNeighbour(std::shared_ptr<SquareGrid> sg) : sgrid_(sg)
{
}

std::vector<std::tuple<SquareCell, double>> GetSquareGridNeighbour::operator()(SquareCell tile)
{
//     std::vector<std::tuple<FieldTile, double>> adjacent_tiles;

// #ifdef ALLOW_DIAGONAL_MOVE
//     for (int32_t j = tile.position_y_ - 1; j <= tile.position_y_ + 1; ++j)
//         for (int32_t i = tile.position_x_ - 1; i <= tile.position_x_ + 1; ++i)
//         {
//             if (i == tile.position_x_ && j == tile.position_y_)
//                 continue;

//             if (j >= 0 && j < field_->SizeY() &&
//                 i >= 0 && i < field_->SizeX())
//             {
//                 FieldTile ntile(i, j, field_);

//                 if (ntile.IsLaneConstrained())
//                     continue;

//                 int32_t x_err = ntile.position_x_ - tile.position_x_;
//                 int32_t y_err = ntile.position_y_ - tile.position_y_;
//                 double dist = std::sqrt(x_err * x_err + y_err * y_err);

//                 double edge_cost = ntile.GetCollisionThreat() - tile.GetCollisionThreat() + dist * 0.001;

//                 adjacent_tiles.emplace_back(ntile, edge_cost);
//             }
//         }
// #else
//     int32_t pos_x[4];
//     int32_t pos_y[4];

//     pos_x[0] = tile.position_x_ - 1;
//     pos_y[0] = tile.position_y_;

//     pos_x[1] = tile.position_x_ + 1;
//     pos_y[1] = tile.position_y_;

//     pos_x[2] = tile.position_x_;
//     pos_y[2] = tile.position_y_ - 1;

//     pos_x[3] = tile.position_x_;
//     pos_y[3] = tile.position_y_ + 1;

//     for (int i = 0; i < 4; ++i)
//     {
//         if (pos_y[i] >= 0 && pos_y[i] < field_->SizeY() && pos_x[i] >= 0 && pos_x[i] < field_->SizeX())
//         {
//             FieldTile ntile(pos_x[i], pos_y[i], field_);
//             if (ntile.IsLaneConstrained())
//                 continue;
//             double edge_cost = ntile.GetCollisionThreat() - tile.GetCollisionThreat();
//             adjacent_tiles.emplace_back(ntile, edge_cost);
//         }
//     }
// #endif

//     return adjacent_tiles;
}
