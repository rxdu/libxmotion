/*
 * square_grid.h
 *
 *  Created on: Jan 28, 2016
 *      Author: rdu
 */

#ifndef SRC_MAP_SQUARE_GRID_H_
#define SRC_MAP_SQUARE_GRID_H_

#include <map>
#include <vector>
#include <cstdint>
#include "common/common_types.h"

namespace srcl_ctrl{

struct SquareCell{
	SquareCell():node_id_(0){
		occu_ = OccupancyType::FREE;

		bbox_.x.min = 0;
		bbox_.x.max = 0;
		bbox_.y.min = 0;
		bbox_.y.max = 0;
	}

	SquareCell(uint64_t id, uint32_t row, uint32_t col, OccupancyType occupancy):
		node_id_(id),occu_(occupancy)
	{
		index_.x = col;
		index_.y = row;

		bbox_.x.min = 0;
		bbox_.x.max = 0;
		bbox_.y.min = 0;
		bbox_.y.max = 0;
	}

	SquareCell(uint64_t id, uint32_t row, uint32_t col, BoundingBox bbox, OccupancyType occupancy):
		node_id_(id),occu_(occupancy)
	{
		index_.x = col;
		index_.y = row;

		bbox_ = bbox;

		location_.x = bbox_.x.min + (bbox_.x.max - bbox_.x.min)/2;
		location_.y = bbox_.y.min + (bbox_.y.max - bbox_.y.min)/2;
	}

	const uint64_t node_id_;
	Position2D index_;
	Position2D location_;
	OccupancyType occu_;
	BoundingBox bbox_;
};

class SquareGrid{
public:
	SquareGrid(uint32_t row_num, uint32_t col_num, uint32_t cell_size);
	~SquareGrid();

public:
	std::map<uint64_t, SquareCell*> cells_;

public:
	uint32_t row_size_;
	uint32_t col_size_;
	uint32_t cell_size_;

private:
	BoundingBox CalcBoundingBox(uint64_t id);

public:
	void SetCellOccupancy(uint32_t row, uint32_t col, OccupancyType occ);
	void SetCellOccupancy(uint64_t id, OccupancyType occ);
	uint32_t GetIDFromPosition(uint32_t row, uint32_t col);
	std::vector<SquareCell*> GetNeighbours(uint64_t id);
	std::vector<SquareCell*> GetNeighbours(uint64_t id, bool allow_diag);
};

}


#endif /* SRC_MAP_SQUARE_GRID_H_ */
