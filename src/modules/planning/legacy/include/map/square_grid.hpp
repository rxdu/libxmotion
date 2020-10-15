/*
 * square_grid.hpp
 *
 * Created on: Jan 28, 2016
 * Description:
 *
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef SQUARE_GRID_HPP
#define SQUARE_GRID_HPP

#include <map>
#include <vector>
#include <cstdint>

#include "navtypes/navtypes.hpp"

namespace ivnav
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

class SquareCell
{
  public:
	SquareCell(int64_t id, int32_t col, int32_t row, OccupancyType occupancy);
	~SquareCell() = default;

	// abstract square cell attributes
	int64_t id_;
	Position2Di coordinate_;
	OccupancyType occupancy_;

	// additional informaion when associated with an image map
	Position2Dd position_;
	BoundingBox<int32_t> bounding_box_;

	int64_t GetUniqueID() const { return id_; }
	void UpdateMapInfo(int32_t row_size, int32_t col_size, double side_size, int32_t pixel_per_meter);
	void PrintInfo();
};

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

	std::vector<std::vector<SquareCell *>> grid_cells_;

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
