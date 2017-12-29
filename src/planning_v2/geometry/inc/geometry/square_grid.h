/*
 * square_grid.h
 *
 * Created on: Jan 28, 2016
 * Description:
 *
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef SQUARE_GRID_H
#define SQUARE_GRID_H

#include <map>
#include <vector>
#include <cstdint>

#include <common/librav_types.h>

namespace librav {

class SquareCell
{
public:
	SquareCell(uint64_t id, int32_t col, int32_t row, OccupancyType occupancy):
		id_(id),
		occupancy_(occupancy)
	{
		assert((col >= 0 && row >= 0));

		index_.x = col;
		index_.y = row;
	}
	~SquareCell() = default;

	uint64_t id_;
	Position2Di index_;
	Position2Di position_;
	OccupancyType occupancy_;

	void PrintInfo()
	{		
		std::string occu;
		switch(occupancy_)
		{
			case OccupancyType::FREE:
			occu = "free";
			break;
			case OccupancyType::OCCUPIED:
			occu = "occupied";
			break;
			case OccupancyType::UNKONWN:
			occu = "unknown";
			break;
		}
		std::cout << "Cell - id: " << id_ << ", index: (" << index_.x << "," << index_.y << ")" << ", occupancy: " << occu << std::endl;
	}
};

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
class SquareGrid
{
public:
	SquareGrid(int32_t col_num, int32_t row_num);
	~SquareGrid();

	typedef SquareCell node_type;

	int32_t row_size_;
	int32_t col_size_;
	std::map<uint64_t, SquareCell*> cells_;

public:
	void SetCellOccupancy(uint32_t col, uint32_t row, OccupancyType occ);
	void SetCellOccupancy(uint64_t id, OccupancyType occ);
	uint64_t GetIDFromIndex(uint32_t col, uint32_t row);
	SquareCell *GetCellFromID(uint64_t id);
	std::vector<SquareCell *> GetNeighbours(uint64_t id, bool allow_diag = false);
};
}

#endif /* SQUARE_GRID_H */
