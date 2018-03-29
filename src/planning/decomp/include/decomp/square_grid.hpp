/* 
 * square_grid.hpp
 * 
 * Created on: Mar 28, 2018 23:07
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef SQUARE_GRID_HPP
#define SQUARE_GRID_HPP

#include <cstdint>

namespace librav
{
/*
 * Coordinate System:
 *		y
 *	^	^
 *	^	|
 *		|
 * row	|
 *		|
 *		|
 *	v	|
 *  v	origin ------------------> x
 *		<<		   column       >>
 */
struct SquareCell
{
    bool occupied = false;
};

template <typename TileType>
class SquareGrid
{
  public:
    SquareGrid(int32_t row_num, int32_t col_num, double cell_size = 0.1, int32_t pixel_per_meter = 100);
    ~SquareGrid();

    typedef SquareCell node_type;

    int32_t row_size_;
    int32_t col_size_;
    double cell_size_;
    int32_t pixel_per_meter_;

    std::vector<std::vector<TileType>> grid_tiles_;

  public:
    void SetCellOccupancy(int32_t x_col, int32_t y_row, OccupancyType occ);
    void SetCellOccupancy(int64_t id, OccupancyType occ);

    Position2Di GetCoordinateFromID(int64_t id);
    int64_t GetIDFromCoordinate(int32_t x_col, int32_t y_row);

    SquareCell *GetCellFromID(int64_t id);
    std::vector<SquareCell *> GetNeighbours(int32_t x_col, int32_t y_row, bool allow_diag);
    std::vector<SquareCell *> GetNeighbours(int64_t id, bool allow_diag = false);
};
}

#endif /* SQUARE_GRID_HPP */
