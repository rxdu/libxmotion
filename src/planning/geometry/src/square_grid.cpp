/*
 * square_grid.cpp
 *
 *  Created on: Jan 28, 2016
 *      Author: rdu
 */

//#include <iostream>
#include "geometry/square_grid.h"

using namespace librav;

SquareGrid::SquareGrid(uint32_t col_num, uint32_t row_num, uint32_t cell_size):
		row_size_(row_num), col_size_(col_num), cell_size_(cell_size),
		img_offset_x_(0), img_offset_y_(0)
{
	for(uint32_t j = 0; j < row_num; j++)
		for(uint32_t i = 0; i < col_num; i++)
		{
			uint64_t new_id = j * col_num + i;
			SquareCell* new_cell = new SquareCell(new_id, i, j, CalcBoundingBox(new_id), OccupancyType::FREE);
			cells_[new_id] = new_cell;
		}
}

SquareGrid::SquareGrid(uint32_t col_num, uint32_t row_num, uint32_t cell_size, int64_t img_offset_x, int64_t img_offset_y):
		row_size_(row_num), col_size_(col_num), cell_size_(cell_size),
		img_offset_x_(img_offset_x), img_offset_y_(img_offset_y)
{
	for(uint32_t j = 0; j < row_num; j++)
		for(uint32_t i = 0; i < col_num; i++)
		{
			uint64_t new_id = j * col_num + i;
			SquareCell* new_cell = new SquareCell(new_id, i, j, CalcBoundingBox(new_id, img_offset_x_, img_offset_y_), OccupancyType::FREE);
			cells_[new_id] = new_cell;
		}
}

SquareGrid::~SquareGrid(){
	for(auto itm = cells_.begin(); itm != cells_.end(); itm++)
		delete itm->second;
}

void SquareGrid::SetCellOccupancy(uint32_t col, uint32_t row, OccupancyType occ)
{
	SetCellOccupancy(col+row*col_size_, occ);
}
void SquareGrid::SetCellOccupancy(uint64_t id, OccupancyType occ)
{
	cells_[id]->occu_ = occ;
}

uint64_t SquareGrid::GetIDFromIndex(uint32_t col, uint32_t row)
{
	return row * col_size_ + col;
}

uint64_t SquareGrid::GetIDFromPosition(uint32_t x, uint32_t y)
{
	uint32_t row, col;

	col = (x - img_offset_x_) / cell_size_;
	row = (y - img_offset_y_) / cell_size_;

	return GetIDFromIndex(col, row);
}

SquareCell* SquareGrid::GetCellFromID(uint64_t id)
{
	auto it = cells_.find(id);

	if(it != cells_.end())
		return (*it).second;
	else
		return nullptr;
}

BoundingBox SquareGrid::CalcBoundingBox(uint64_t id)
{
	BoundingBox bbox;
	uint32_t x,y;
	x = id%col_size_;
	y = row_size_ - id/col_size_ - 1;
	bbox.x.min = x*cell_size_;
	bbox.x.max = bbox.x.min + cell_size_ - 1;
	bbox.y.min = y*cell_size_;
	bbox.y.max = bbox.y.min + cell_size_ - 1;

	return bbox;
}

BoundingBox SquareGrid::CalcBoundingBox(uint64_t id, int64_t img_offset_x, int64_t img_offset_y)
{
	BoundingBox bbox;
	uint32_t x,y;
	x = id%col_size_;
	y = id/col_size_;
	bbox.x.min = x*cell_size_ + img_offset_x;
	bbox.x.max = bbox.x.min + cell_size_ - 1;
	bbox.y.min = y*cell_size_ + img_offset_y;
	bbox.y.max = bbox.y.min + cell_size_ - 1;

	return bbox;
}

std::vector<SquareCell*> SquareGrid::GetNeighbours(uint64_t id, bool allow_diag)
{
	std::vector<SquareCell*> neighbours;

	uint32_t x,y;
	x = cells_[id]->index_.x;
	y = cells_[id]->index_.y;

	// not consider diagonal cells
	if(allow_diag)
	{
		Position2Di pos[8];

		pos[0].x = x - 1;
		pos[0].y = y - 1;

		pos[1].x = x;
		pos[1].y = y - 1;

		pos[2].x = x + 1;
		pos[2].y = y - 1;

		pos[3].x = x - 1;
		pos[3].y = y;

		pos[4].x = x + 1;
		pos[4].y = y;

		pos[5].x = x - 1;
		pos[5].y = y + 1;

		pos[6].x = x;
		pos[6].y = y + 1;

		pos[7].x = x + 1;
		pos[7].y = y + 1;

		for(int i = 0; i < 8; i++)
		{
			if(pos[i].x < col_size_ && pos[i].y < row_size_)
				neighbours.push_back(cells_[pos[i].y * col_size_ + pos[i].x]);
		}
	}
	else
	{
		Position2Di pos[4];

		pos[0].x = x;
		pos[0].y = y + 1;

		pos[1].x = x;
		pos[1].y = y - 1;

		pos[2].x = x + 1;
		pos[2].y = y;

		pos[3].x = x - 1;
		pos[3].y = y;

		for(int i = 0; i < 4; i++)
		{
			if(pos[i].x < col_size_ && pos[i].y < row_size_)
				neighbours.push_back(cells_[pos[i].y * col_size_ + pos[i].x]);
		}
	}

	return neighbours;
}

std::vector<SquareCell*> SquareGrid::GetNeighboursWithinRange(uint64_t id, uint32_t cell_range)
{
	std::vector<SquareCell*> neighbours;

	uint32_t x,y;
	x = cells_[id]->index_.x;
	y = cells_[id]->index_.y;

	uint32_t xmin,xmax,ymin,ymax;

	if(x < cell_range)
		xmin = 0;
	else
		xmin = x - cell_range;
	if(x + cell_range >= col_size_)
		xmax = col_size_ - 1;
	else
		xmax = x + cell_range;

	if(y < cell_range)
		ymin = 0;
	else
		ymin = y - cell_range;
	if(y + cell_range >= row_size_)
		ymax = row_size_ - 1;
	else
		ymax = y + cell_range;

	for(int64_t i = xmin; i <= xmax; i++)
		for(int64_t j = ymin; j <= ymax; j++) {
			if(i == x && j == y)
				continue;

			neighbours.push_back(cells_[j * col_size_ + i]);
		}

	return neighbours;
}

