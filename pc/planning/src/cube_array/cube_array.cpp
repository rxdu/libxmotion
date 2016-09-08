/*
 * cube_array.cpp
 *
 *  Created on: Sep 8, 2016
 *      Author: rdu
 */

#include <iostream>

#include "cube_array/cube_array.h"

using namespace srcl_ctrl;

CubeArray::CubeArray(uint32_t row_num, uint32_t col_num, uint32_t height_num, double cube_size):
		row_size_(row_num),
		col_size_(col_num),
		hei_size_(height_num),
		cube_size_(cube_size)
{
	cubes_.resize(row_size_*col_size_*hei_size_);

	for(uint32_t hei = 0; hei < hei_size_; hei++)
		for(uint32_t row = 0; row < row_size_; row++)
			for(uint32_t col = 0; col < col_size_; col++)
			{
				uint64_t id = GetIDFromIndex(row, col, hei);
				cubes_[id].data_id_ = id;
				cubes_[id].index_.x = row;
				cubes_[id].index_.y = col;
				cubes_[id].index_.z = hei;
				cubes_[id].location_.x = cube_size_/2.0 + row * cube_size_;
				cubes_[id].location_.y = cube_size_/2.0 + col * cube_size_;
				cubes_[id].location_.z = cube_size_/2.0 + hei * cube_size_;
				cubes_[id].occu_ = OccupancyType::OCCUPIED;

//				std::cout << "id: " << id << ", coordinate: " << cubes_[id].location_.x << " , "
//						<< cubes_[id].location_.y << " , "
//						<< cubes_[id].location_.z << std::endl;
			}
}

CubeArray::~CubeArray()
{

}

uint64_t CubeArray::GetIDFromIndex(uint32_t row, uint32_t col, uint32_t hei)
{
	uint64_t id = 0;

	id = hei * (row_size_*col_size_) + row * row_size_ + col;

	return id;
}

uint64_t CubeArray::GetIDFromPosition(double x, double y, double z)
{
	uint32_t row, col, hei;

	if(x < 0)
		x = 0;
	if(y < 0)
		y = 0;
	if(z < 0)
		z = 0;

	if(x > row_size_ * cube_size_)
		x = row_size_ * cube_size_;
	if(y > col_size_ * cube_size_)
		y = col_size_ * cube_size_;
	if(z > hei_size_ * cube_size_)
		z = hei_size_ * cube_size_;

	row = static_cast<uint32_t>(x/cube_size_);
	col = static_cast<uint32_t>(y/cube_size_);
	hei = static_cast<uint32_t>(z/cube_size_);

	return GetIDFromIndex(row, col, hei);
}

void CubeArray::UpdateCubeOccupancy(double x, double y, double z, OccupancyType oc_type)
{
	uint64_t id = GetIDFromPosition(x,y,z);

	cubes_[id].occu_ = oc_type;
}

std::vector<uint64_t> CubeArray::GetNeighbours(uint64_t id)
{
	std::vector<uint64_t> neighbours;

	int32_t row, col, hei;
	row = cubes_[id].index_.x;
	col = cubes_[id].index_.y;
	hei = cubes_[id].index_.z;

	// do not allow diagonal movement
	if(row - 1 >= 0)
		neighbours.push_back(GetIDFromIndex(row - 1, col, hei));
	if(row + 1 < row_size_)
			neighbours.push_back(GetIDFromIndex(row + 1, col, hei));
	if(col - 1 >= 0)
		neighbours.push_back(GetIDFromIndex(row, col - 1, hei));
	if(col + 1 < col_size_)
		neighbours.push_back(GetIDFromIndex(row, col + 1, hei));
	if(hei - 1 >= 0)
		neighbours.push_back(GetIDFromIndex(row, col, hei - 1));
	if(hei + 1 < hei_size_)
		neighbours.push_back(GetIDFromIndex(row, col, hei + 1));

	return neighbours;
}
