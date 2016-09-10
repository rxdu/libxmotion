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
		row_offset_(0),
		col_offset_(0),
		hei_offset_(0),
		cube_size_(cube_size)
{
	cubes_.resize(row_size_*col_size_*hei_size_);

	for(uint32_t hei = 0; hei < hei_size_; hei++)
		for(uint32_t col = 0; col < col_size_; col++)
			for(uint32_t row = 0; row < row_size_; row++)
			{
				uint64_t id = GetIDFromIndex(row, col, hei);
				cubes_[id].data_id_ = id;
				cubes_[id].index_.x = row;
				cubes_[id].index_.y = col;
				cubes_[id].index_.z = hei;
				cubes_[id].location_.x = cube_size_/2.0 + static_cast<int32_t>(row) * cube_size_;
				cubes_[id].location_.y = cube_size_/2.0 + static_cast<int32_t>(col) * cube_size_;
				cubes_[id].location_.z = cube_size_/2.0 + static_cast<int32_t>(hei) * cube_size_;
				cubes_[id].occu_ = OccupancyType::OCCUPIED;

//				std::cout << "id: " << id <<
//						", coordinate: "
//						<< cubes_[id].location_.x << " , "
//						<< cubes_[id].location_.y << " , "
//						<< cubes_[id].location_.z<<
//						", index: "
//						<< cubes_[id].index_.x << " , "
//						<< cubes_[id].index_.y << " , "
//						<< cubes_[id].index_.z << std::endl;
			}
}

void CubeArray::SetOriginOffset(int32_t row_offset, int32_t col_offset, int32_t hei_offset)
{
	row_offset_ = row_offset;
	col_offset_ = col_offset;
	hei_offset_ = hei_offset;

	for(uint32_t hei = 0; hei < hei_size_; hei++)
		for(uint32_t col = 0; col < col_size_; col++)
			for(uint32_t row = 0; row < row_size_; row++)
			{
				uint64_t id = GetIDFromIndex(row, col, hei);
				cubes_[id].location_.x = cube_size_/2.0 + (static_cast<int32_t>(row) - row_offset_) * cube_size_;
				cubes_[id].location_.y = cube_size_/2.0 + (static_cast<int32_t>(col) - col_offset_) * cube_size_;
				cubes_[id].location_.z = cube_size_/2.0 + (static_cast<int32_t>(hei) - col_offset_) * cube_size_;

//				std::cout << "id: " << id <<
//						", coordinate: "
//						<< cubes_[id].location_.x << " , "
//						<< cubes_[id].location_.y << " , "
//						<< cubes_[id].location_.z<<
//						", index: "
//						<< cubes_[id].index_.x << " , "
//						<< cubes_[id].index_.y << " , "
//						<< cubes_[id].index_.z << std::endl;
			}
}

uint64_t CubeArray::GetIDFromIndex(uint32_t row, uint32_t col, uint32_t hei)
{
	uint64_t id = 0;

	id = hei * (row_size_*col_size_) + col*row_size_ + row;

	return id;
}

uint64_t CubeArray::GetIDFromPosition(double x, double y, double z)
{
	uint32_t row, col, hei;

	if(x < - row_offset_ * cube_size_)
		x = -row_offset_ * cube_size_;
	if(y < -col_offset_ * cube_size_)
		y = -col_offset_ * cube_size_;
	if(z < -hei_offset_ * cube_size_)
		z = -hei_offset_ * cube_size_;

	// avoid the top right corner
	if(x > (static_cast<int32_t>(row_size_)-row_offset_) * cube_size_)
		x = (static_cast<int32_t>(row_size_)-row_offset_) * cube_size_ - cube_size_/10.0;
	if(y > (static_cast<int32_t>(col_size_)-col_offset_) * cube_size_)
		y = (static_cast<int32_t>(col_size_)-col_offset_) * cube_size_ - cube_size_/10.0;
	if(z > (static_cast<int32_t>(hei_size_)-hei_offset_) * cube_size_)
		z = (static_cast<int32_t>(hei_size_)-hei_offset_) * cube_size_ - cube_size_/10.0;

//	std::cout << "(x, y, z): " << x << " , " << y << " , " << z << std::endl;
//	std::cout << "divide: " << x/cube_size_ << std::endl;
	row = static_cast<int32_t>((x + row_offset_*cube_size_)/cube_size_);
	col = static_cast<int32_t>((y + col_offset_*cube_size_)/cube_size_);
	hei = static_cast<int32_t>((z + hei_offset_*cube_size_)/cube_size_);

//	std::cout << "(row, col, hei): " << row << " , " << col << " , " << hei << std::endl;
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

	int64_t row, col, hei;
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
