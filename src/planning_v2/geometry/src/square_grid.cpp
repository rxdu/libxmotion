/* 
 * square_grid.cpp
 * 
 * Created on: Jan 28, 2016
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

//#include <iostream>
#include "geometry/square_grid.h"

using namespace librav;

// Note: Assertions are for debugging purpose. Since assert() will be expanded as void if "NDEBUG" is defined, 
// 	you should not rely on it to do online condition checking in Release code.
#define ASSERT_INDEX_RANGE(col, row) \
	assert(col >= 0 && row >= 0 && col < col_size_ && row < row_size_)

#define ASSERT_ID_RANGE(id) \
	assert(id >= 0 && id < col_size_ * row_size_)


SquareGrid::SquareGrid(int32_t col_num, int32_t row_num) : row_size_(row_num), col_size_(col_num)
{
	assert((col_num > 0 && row_num > 0));

	for (int32_t j = 0; j < row_num; j++)
		for (int32_t i = 0; i < col_num; i++)
		{
			uint64_t new_id = j * col_num + i;
			SquareCell *new_cell = new SquareCell(new_id, i, j, OccupancyType::FREE);
			cells_[new_id] = new_cell;
		}
}

// This should be the only place to delete SquareGrid objects
SquareGrid::~SquareGrid()
{
	for (auto itm = cells_.begin(); itm != cells_.end(); itm++)
		delete itm->second;
}

void SquareGrid::SetCellOccupancy(uint32_t col, uint32_t row, OccupancyType occ)
{
	ASSERT_INDEX_RANGE(col, row);

	SetCellOccupancy(col + row * col_size_, occ);
}

void SquareGrid::SetCellOccupancy(uint64_t id, OccupancyType occ)
{
	ASSERT_ID_RANGE(id);

	cells_[id]->occupancy_ = occ;
}

uint64_t SquareGrid::GetIDFromIndex(uint32_t col, uint32_t row)
{
	ASSERT_INDEX_RANGE(col, row);

	return row * col_size_ + col;
}

SquareCell *SquareGrid::GetCellFromID(uint64_t id)
{
	ASSERT_ID_RANGE(id);

	return cells_[id];
}

std::vector<SquareCell *> SquareGrid::GetNeighbours(uint64_t id, bool allow_diag)
{
	ASSERT_ID_RANGE(id);

	std::vector<SquareCell *> neighbours;

	int32_t xc, yc;
	xc = cells_[id]->index_.x;
	yc = cells_[id]->index_.y;

	// not consider diagonal cells
	if (allow_diag)
	{
		for(int32_t x = xc - 1; x <= xc + 1; ++x)
			for(int32_t y = yc - 1; y <= yc + 1; ++y)
			{
				if(x == xc && y == yc)
					continue;

				if (x > 0 && x < col_size_ &&
					 y > 0 && y < row_size_)
					neighbours.push_back(cells_[GetIDFromIndex(x,y)]);
			}
	}
	else
	{
		Position2Di pos[4];

		pos[0].x = xc;
		pos[0].y = yc + 1;

		pos[1].x = xc;
		pos[1].y = yc - 1;

		pos[2].x = xc + 1;
		pos[2].y = yc;

		pos[3].x = xc - 1;
		pos[3].y = yc;

		for (int i = 0; i < 4; i++)
		{
			if (pos[i].x > 0 && pos[i].x < col_size_ &&
				pos[i].y > 0 && pos[i].y < row_size_)
				neighbours.push_back(cells_[GetIDFromIndex(pos[i].x,pos[i].y)]);
		}
	}

	return neighbours;
}
